import random
# from pprint import *
import matplotlib.pyplot as plt
import json
#from pkg_scheduler.sp_comsat.edges_for_graphs import *
from edges_for_graphs import *
import networkx as nx
#from pkg_scheduler.sp_comsat.support_functions import get_divisors
from support_functions import get_divisors
# Utility function to print the elements of an array
def printArr(arr, n):
    for i in range(n):
        print(arr[i], end=" ")

# m random non-negative integers
# whose sum is n
def randomList(m, n):
    # Create an array of size m where
    # every element is initialized to 0
    arr = [0] * m

    # To make the sum of the final list as n
    for i in range(n):
        # Increment any random element
        # from the array by 1
        arr[random.randint(0, n) % m] += 1

    # Print the generated list
    return arr

def make_an_instance(nodes,num_vehicles,num_jobs,edges_reduction,Big_number,SEED):
        
        scaling_factor = 4

        random.seed(SEED)
        grid_length = int(random.choice(list(get_divisors(nodes))[1:-1]))
        grid_width = int(nodes / grid_length)
        # print(grid_width,grid_length)
        grid = nx.grid_2d_graph(grid_width,grid_length)
        new_nodes = {i:index for index,i in enumerate(grid)}

        graph = nx.DiGraph()
        for i in grid.nodes:
            graph.add_node(new_nodes[i])
        edge_candidates = []
        for i in grid.edges:
            edge_candidates.append((new_nodes[i[0]],new_nodes[i[1]]))
            edge_candidates.append((new_nodes[i[1]],new_nodes[i[0]]))
        actual_edges = random.sample(edge_candidates,round(len(edge_candidates)*edges_reduction))
        graph.add_edges_from(actual_edges)



        # graph= nx.fast_gnp_random_graph(nodes, edges_reduction, directed=True,seed=int(SEED))
        # nodes_dict = nx.random_layout(graph,seed=int(SEED))
        nodes_dict = {new_nodes[i]: {
            'pos': (i[0]*scaling_factor,i[1]*scaling_factor),
            'next': [ str(j[1]) for j in graph.out_edges(new_nodes[i])]
        } for i in grid}
        # DOUBT: why do we multiply by scaling_factor above?? and how is it decided??

        nx.set_node_attributes(graph,nodes_dict)
        nx.draw(graph, nx.get_node_attributes(graph, 'pos'), with_labels=True)
        plt.show()
        print("The Graph is strongly connected...", nx.is_strongly_connected(graph))


        # for i in graph.edges:
        #     graph[i[0]][i[1]]['weight'] = '1'
        #     graph[i[0]][i[1]]['capacity'] = '2'
        # print(graph.edges.data())
        NVJ = str(nodes) + str(num_vehicles) + str(num_jobs)

        print('making instance {}_{}_{}_{}_{}_{}'.format(nodes,num_vehicles,num_jobs,edges_reduction,Big_number,SEED))

        to_dump = {}


        # range of possible operating range as a function of the number of nodes
        ####### COME UP WITH SOMETHING BETTER PLEASE
        # DOUBT: Need explaination on how it was decided? 
        autonomy_range = [(9 + nodes) * scaling_factor, (18 + nodes) * scaling_factor]

        nodes_list = {i:{
                        'x':nodes_dict[i]['pos'][0],
                        'y': nodes_dict[i]['pos'][1],
                        'next': nodes_dict[i]['next']
                        }
             for i in nodes_dict}
        depots = random.sample(list(nodes_list.keys()),num_vehicles)
        start_list = {
                        "{}".format(vehicle): str(depots[vehicle])
                        for vehicle in range(num_vehicles)
                    }

        test_data = {
            'Big_number' : Big_number,
            'Autonomy' : random.randint(autonomy_range[0],autonomy_range[1]),
            'charging_coefficient' : random.randint(1,3),
            'nodes' : nodes_list,
            'start_list' : start_list,
            'charging_stations': [str(i) for i in depots], #random.sample(nodes_list,k=int(nodes/5)),
            'hub_nodes' : [] #[str(i) for i in depots] no hub nodes allowed in GPSS
        }


        to_dump.update({'test_data':test_data})

        ves = randomList(3, num_vehicles)

        vehicles =  {
            "{}".format(vehicle):{"units": 1}
            for vehicle in range(num_vehicles)
        }


        to_dump.update({'ATRs':vehicles})

        # DOUBT: Please explain the reasoning behind the logic
        vehicles_per_job = [random.randint(1,sum([
                                1  if vehicles[j]['units'] > 0 else 0 for j in vehicles
                                    ])) for i in range(num_jobs)]

        # print(vehicles_per_job)

        buffer_node_list = [i for i in nodes_list].copy()
        for i in depots:
            buffer_node_list.remove(i)

        jobs_nodes = random.sample(buffer_node_list,k=num_jobs)

        jobs_list = ['task_{}'.format(i) for i in range(num_jobs)]
        # print(jobs_list)
        no_prec_jobs = random.sample(jobs_list,k=int(num_jobs/2))
        prec_jobs = [i for i in jobs_list if i not in no_prec_jobs]
        prec_map = {}
        for i in jobs_list:
            if i in no_prec_jobs:       # DOUBT: Logially speaking, shouldn't this be prec_jobs instead of no_prec_jobs. Technically doesn't matter.
                prec_map.update({
                    i: prec_jobs.pop(random.randrange(len(prec_jobs)))
                })
            else:
                prec_map.update({
                    i:'None'
                })
        # for i in prec_map:
        #     print(i,prec_map[i])

        # edges_for_graph = {(int(i.split(',')[0]),int(i.split(',')[1])):edges[i] for i in edges}

        # with open('paths_container/PL_{}_{}.json'.format(nodes,edges_reduction),'r') as read_file:
        #     paths = json.load(read_file)

        jobs = {}
        # print('bibidi')
        for i,job in enumerate(jobs_list):
            if prec_map[job] != 'None':
                # FOR THE TIME BEING LET'S FORGET ABOUT THE TIME WINDOWS
                # lower_bound = random.randint(5,Big_number-10)
                # upper_bound = min(lower_bound + 10,Big_number-5)
                # time_window = [lower_bound,upper_bound]
                time_window = []
                # .....AND ABOUT PRECEDENCE TOO (WHEN I PUT THIS BACK, MAKE SURE IT DOES NOT LEAD TO UNFEASIBILITY)
                # prec = [prec_map[job]]
                prec = []
            else:
                time_window = []
                prec = []
            jobs.update({
                job:{
                        'location':str(jobs_nodes[i]),
                        'precedence':prec,
                        'TW':time_window,
                        'Service': random.randint(1, 3),
                        'ATR':random.sample([i for i in vehicles if vehicles[i]['units']>0],
                                             k=vehicles_per_job[i]
                                             )
                }
            })



        to_dump.update({'jobs':jobs})
        to_dump.update({'edges': {"{},{}".format(i[0], i[1]): [1, 2] for i in graph.edges}})

        # pprint(test_data)
        # pprint(vehicles)
        # pprint(jobs)
        # pprint(edges)
        
        with open('test_cases/MM_{}_{}_{}_{}_{}_{}.json'.format(
                                                        nodes, num_vehicles, num_jobs,
                                                        edges_reduction,
                                                        Big_number,
                                                        SEED
                                                        ), 'w+') as write_file:
                json.dump(to_dump,write_file,indent=4)

if __name__ == "__main__":
    ################ MAKE A SINGLE INSTANCE
    # make_an_instance(15,3,8,0.2,20,21)
    make_an_instance(27,10,10,0.7,7,7)
    # make_an_instance(1000,10,25,0.2,500,21)

    ################ MAKE SEVERAL INSTANCES USING FOR LOOPS BASED ON :
    # nodes   num_vehicles   num_jobs   edges_reduction   Big_number   SEED

    # for NVJ in [[15,3,5],[25,4,7]]:
    #     for edge_reduction in [0,25,50]:
    #         for Big_Num in [20, 25, 30, 40, 50, 60]:
    #             for i in range(5,10):
    #                 make_an_instance(NVJ[0],NVJ[1],NVJ[2],edge_reduction,Big_Num,i)

    # SOME LARGER INSTANCES
    # for SEED in range(5,10):
    #     for Big_Num in [40,70,100,150,200,300]:
    #         for NVJ in [[35,9,12],[35,11,15]]: #[15,3,5],[25,4,7]
    #             for edge_reduction in [0,25,50]:
    #                 make_an_instance(NVJ[0],NVJ[1],NVJ[2],edge_reduction,Big_Num,SEED)

    ####################################################
    # THIS FUNCTION GENERATES THE SHORTEST PATHS BETWEEN ANY TWO NODES IN A GRAPH, WHERE
    # THE INPUT GRAPH IS ONE OF THE LAYOUTS USED FOR THE BENCHMARK OF THE CF-EVRP
    ####################################################
    # def path_generator(nodes,edges_reduction):
    #
    #     print(nodes,edges_reduction)
    #
    #     to_dump = {}
    #
    #     if nodes == 15:
    #         if edges_reduction == 0:
    #             edges = edges_15_100
    #         elif edges_reduction == 25:
    #             edges = edges_15_75
    #         elif edges_reduction == 50:
    #             edges = edges_15_50
    #         else:
    #             raise ValueError('WRONG EDGE REDUCTION')
    #     elif nodes == 25:
    #         if edges_reduction == 0:
    #             edges = edges_25_100
    #         elif edges_reduction == 25:
    #             edges = edges_25_75
    #         elif edges_reduction == 50:
    #             edges = edges_25_50
    #         else:
    #             raise ValueError('WRONG EDGE REDUCTION')
    #     elif nodes == 35:
    #         if edges_reduction == 0:
    #             edges = edges_35_100
    #         elif edges_reduction == 25:
    #             edges = edges_35_75
    #         elif edges_reduction == 50:
    #             edges = edges_35_50
    #         else:
    #             raise ValueError('WRONG EDGE REDUCTION')
    #     else:
    #         raise ValueError('WRONG NUMBER OF NODES')
    #
    #     nodes_list = [i for i in range(nodes)]
    #     edges_for_graph = {(int(i.split(',')[0]), int(i.split(',')[1])): edges[i] for i in edges}
    #
    #     graph = make_graph(
    #         nodes_list,
    #         edges_for_graph
    #     )
    #
    #     paths = {
    #         # '{},{}'.format(i,j): find_shortest_path(graph, i, j)
    #         # for i in nodes_list
    #         # for j in nodes_list
    #     }
    #     for i in nodes_list:
    #         for j in nodes_list:
    #             print(i,j)
    #             paths.update({'{},{}'.format(i,j): find_shortest_path(graph, i, j)})
    #
    #     to_dump.update(paths)
    #
    #     with open('paths_container/PL_{}_{}.json'.format(
    #                                                     nodes,
    #                                                     edges_reduction
    #                                                     ), 'w+') as write_file:
    #             json.dump(to_dump, write_file, indent=4)
    #########################  NOW LET'S USE THIS FUNCTION TO MAKE PATHS
    # for nodes in [15,25,35]:
    #     for edge_reduction in [0,25,50]:
    #         path_generator(nodes,edge_reduction)



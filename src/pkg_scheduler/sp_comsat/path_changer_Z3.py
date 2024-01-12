from z3 import *
from classes import Path

def changer(graph, paths_combo, previous_paths=[]):

    use_node = [[[Bool('route_{}_pair_{}_node_{}'.format(route,pair,i)) for i in graph.nodes]
                 for pair in paths_combo[route]] for route in paths_combo]
    use_edge = [[[Bool('route_{}_pair_{}_edge_{}'.format(route,pair,i)) for i in graph.edges]
                 for pair in paths_combo[route]] for route in paths_combo]
    # print('variables')

    start_and_end_are_true = [
        use_node[route_i][index][node_index]
            for route_i,route in enumerate(paths_combo)
                for index,pair in enumerate(paths_combo[route])
                    for node_index,node in enumerate(graph.nodes)
                        if node in pair
    ]
    # print('start and end true')

    only_one_edge_for_start = [
        PbEq([(use_edge[route_i][pair_index][edge_index],1) for edge_index,i in enumerate(graph.edges)
                                if i in graph.out_edges(pair[0])]
             ,1)
        for route_i,route in enumerate(paths_combo)
        for pair_index, pair in enumerate(paths_combo[route])
    ]
    # print('only one edge for start')

    # -- 31.B Exactly zero incoming edges are used for the start node of each route (not every pair, only first)
    # -- !! THIS CONSTRAINT WASN'T PRESENT IN THE ORIGINAL MODEL
    zero_incoming_edge_for_start = [
        PbEq([(use_edge[route_i][pair_index][edge_index], 1) for edge_index, i in enumerate(graph.edges)
              if i in graph.in_edges(pair[0])]
             , 0)
        for route_i,route in enumerate(paths_combo)
        for pair_index, pair in enumerate(paths_combo[route])
        # if pair_index == 0
    ]
    # print('zero incoming edges for start')

    only_one_edge_for_end = [
        PbEq([(use_edge[route_i][pair_index][edge_index],1) for edge_index,i in enumerate(graph.edges)
                                if i in graph.in_edges(pair[1])]
             ,1)
        for route_i,route in enumerate(paths_combo)
        for pair_index, pair in enumerate(paths_combo[route])
    ]
    # print('only one edge for end')

    # -- 32.B Exactly zero outgoing edges are used for the end node of each route (not every pair, only first)
    # -- !! THIS CONSTRAINT WASN'T PRESENT IN THE ORIGINAL MODEL
    zero_outgoing_edge_for_end = [
        PbEq([(use_edge[route_i][pair_index][edge_index], 1) for edge_index, i in enumerate(graph.edges)
              if i in graph.out_edges(pair[1])]
             , 0)
        for route_i,route in enumerate(paths_combo)
        for pair_index, pair in enumerate(paths_combo[route])
        # if pair_index == (len(paths_combo[route])-1)
    ]
    # print('zero outgoing edges for endsss')

    exactly_two_edges = [
        And([
                If(
                use_node[route_i][pair_index][node_index],
                And(
                    PbEq([(use_edge[route_i][pair_index][edge_index],1) for edge_index,j in enumerate(graph.edges)
                                                if j in graph.in_edges(node)
                         ]
                             ,1),
                    PbEq([(use_edge[route_i][pair_index][edge_index],1) for edge_index,j in enumerate(graph.edges)
                                                if j in graph.out_edges(node)
                          ]
                             ,1)
                ),
                And(
                    PbEq([(use_edge[route_i][pair_index][edge_index], 1) for edge_index, j in enumerate(graph.edges)
                                                if j in graph.in_edges(node)
                          ]
                            ,0),
                    PbEq([(use_edge[route_i][pair_index][edge_index], 1) for edge_index, j in enumerate(graph.edges)
                                                if j in graph.out_edges(node)

                          ]
                            ,0)
                )
            )
        for node_index,node in enumerate(graph.nodes)
            if node != pair[0] and node != pair[1]
        ])
        for route_i,route in enumerate(paths_combo)
        for pair_index, pair in enumerate(paths_combo[route])
    ]
    # print('exactly two edges')

    not_both_directions = [
        Implies(
            use_edge[route_i][pair_index][index_i],
            Not(use_edge[route_i][pair_index][index_j])
        )
        for route_i,route in enumerate(paths_combo)
        for pair_index, pair in enumerate(paths_combo[route])
        for index_i,i in enumerate(graph.edges)
        for index_j,j in enumerate(graph.edges)
        if (j[0],j[1]) == (i[1],i[0])
    ]
    # print('not both directions')

    s = Optimize()
    # set_option(verbose=1)
    s.add(
        start_and_end_are_true +
        only_one_edge_for_end +
        only_one_edge_for_start +
        zero_incoming_edge_for_start +
        zero_outgoing_edge_for_end +
        exactly_two_edges +
        not_both_directions
    )

    # print('add to model')

    if previous_paths != []:
        for single_solution in previous_paths:
            s.add(
                Or([
                    Not(use_edge[route_i][pair_index][index])
                       for index, i in enumerate(graph.edges)
                            for route_i,route in enumerate(paths_combo)
                                for pair_index, pair in enumerate(paths_combo[route])
                                    if (route,pair,i) in single_solution
                ])
            )

    # ############# COST FUNCTION TERMS #############
    #
    # cumulative_length = Int('cumulative_length')
    # s.add(
    #       cumulative_length
    #       ==
    #       Sum([
    #         use_edge[route_i][pair_index][edge_index] * graph.get_edge_data(*i)['weight']  #edges[i][0]
    #         for edge_index,i in enumerate(graph.edges)
    #         for route_i,route in enumerate(paths_combo)
    #         for pair_index,_ in enumerate(paths_combo[route])
    #         ])
    #       )
    node_used = [Int('node_{}'.format(node)) for node in graph.nodes]
    s.add([
        node_used[node_index]
        ==
        Sum([
            If(
                Sum([use_node[route_i][index][node_index] for index, pair in enumerate(paths_combo[route])]) > 1,
                1,
                0
            )
            for route_i, route in enumerate(paths_combo)
        ])
        for node_index, node in enumerate(graph.nodes)
    ])
    # print('shared nodes')
    edge_used = [Int('edge_{}'.format(edge)) for edge in graph.edges]
    s.add([
        edge_used[index_i]
        ==
        Sum([
            If(
                Sum([use_edge[route_i][index][index_i] for index, pair in enumerate(paths_combo[route])]) > 1,
                1,
                0
            )
            for route_i, route in enumerate(paths_combo)
        ])
        for index_i, i in enumerate(graph.edges)
    ])

    # print('shared edges')

    sum_nodes_used = Int('sum_nodes_used')
    s.add(
        sum_nodes_used
        ==
        Sum([
            node_used[node]
            for node, _ in enumerate(graph.nodes)
        ])
    )
    sum_edges_used = Int('sum_edges_used')
    s.add(
        sum_edges_used
        ==
        Sum([
            edge_used[edge_index]
            for edge_index, i in enumerate(graph.edges)
        ])
    )

    s.minimize(
        # cumulative_length
        # +
        sum_nodes_used
        +
        sum_edges_used
    )

    PCF = s.check()

    if PCF == sat:

        # print("length: ",s.model()[cumulative_length])
        # print("nodes values: ", s.model()[sum_nodes_used])
        # print("edges values: ", s.model()[sum_edges_used])

        solution = [
                    ( route, pair, i)
                                for index, i in enumerate(graph.edges)
                                    for route_i,route in enumerate(paths_combo)
                                        for pair_index,pair in enumerate(paths_combo[route])
                                            if s.model()[use_edge[route_i][pair_index][index]] == True
                    ]

        # just an intermediate step to convert the paths_changing solution into a format readable by the route_checker
        buffer = {
            route: {
                pair: [sol[2] for sol in solution if sol[0] == route and sol[1] == pair]
                for pair in paths_combo[route]
            }
            for route in paths_combo
        }

        # keep manipulating the format
        new_paths = {}
        for route in buffer:
            new_paths.update({route: {}})
            for pair in buffer[route]:
                sequence = list(buffer[route][pair])
                path = [pair[0]]
                for _ in range(len(sequence)):
                    for i in sequence:
                        if i[0] == path[-1]:
                            path.append(i[1])
                new_paths[route].update({pair: path})

        # last step to get the new paths
        new_paths = {
            route:{
                pair:Path(
                    pair,
                    sum([ graph.get_edge_data(n1,n2)['weight'] for n1,n2 in zip(new_paths[route][pair][:-1],new_paths[route][pair][1:])]),
                    new_paths[route][pair]
                )
                for pair in new_paths[route]
            }
            for route in new_paths
        }

    else:
        solution = []
        new_paths = paths_combo
    return PCF, solution, new_paths
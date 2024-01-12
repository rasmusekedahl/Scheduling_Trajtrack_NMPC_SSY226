from z3 import *
from classes import Route
import math
from time import time as tm

def routing(the_instance, previous_routes = []):

    routing = Optimize()

    # make lists with vehicles and task names to declare gurobi variables
    vehicles = the_instance.ATRs
    tasks = the_instance.tasks
    #compute the shortest paths between any two tasks
    shortest_paths = the_instance.shortest_paths_list()
    # binary variable that states whether a vehicle travels from customer i to customer j
    direct_travel = [[[
        Bool('route_{}_from_{}_to_{}'.format(i, j, k))
        for k in tasks]
        for j in tasks]
        for i in vehicles]

    # continuous variable that keeps track of the arrival of vehicle k at a customer
    customer_served = [Int('{}_is_served'.format(t)) for t in tasks]

    # continuous variable that keeps track of the autonomy left when arriving at a customer
    autonomy_left = [[Int('vehicle_{}_OR_at_{}'.format(i, t)) for t in tasks] for i in vehicles]

    # integer variable that keeps track of the needed to charge
    charging_time = [[Int('vehicle_{}_CT_at_{}'.format(i, t)) for t in tasks] for i in vehicles]

    Z = Int('Z')

    opti = [Z >= customer_served[i] for i,task_i in enumerate(tasks) if task_i.task_type == 'end']

    # 28 the cost function is number of visits to the recharge station while at the same time minimizing
    # the travelling distance
    routing.minimize(
        # the coefficient 100 is used to balance the terms, there could be something better to choose
        100*Sum([
            direct_travel[k][i][j]
            for k, vehicle in enumerate(vehicles)
            for i, task_i in enumerate(tasks)
            for j, task_j in enumerate(tasks)
            if task_j.task_type == 'recharge'
        ])
        # +
        # Sum([
        #     direct_travel[k][i][j] * shortest_paths[(task_i.id,task_j.id)].length
        #     for k, vehicle in enumerate(vehicles)
        #     for i, task_i in enumerate(tasks)
        #     for j, task_j in enumerate(tasks)
        #     if task_i.id != task_j.id
        # ])
        # THIS TERM IS ONLY USED FOR GPSS, where i want the vehicles to return as fast as possible to the depots
        # + Z
    )

    # 29 no travel from and to the same spot
    not_travel_same_spot = [
        Not(direct_travel[k][i][i])
        for k, vehicle in enumerate(vehicles)
        for i, task_i in enumerate(tasks)
    ]

    # 30 no travel to the starting point
    no_travel_to_start = [
        Not(direct_travel[k][i][j])
        for k, vehicle in enumerate(vehicles)
        for i, task_i in enumerate(tasks)
        for j, task_j in enumerate(tasks)
        if task_j.task_type == 'start'
    ]

    # 31 no travel from the end point
    no_travel_from_end = [
        Not(direct_travel[k][i][j])
        for k, vehicle in enumerate(vehicles)
        for i, task_i in enumerate(tasks)
        for j, task_j in enumerate(tasks)
        if task_i.task_type == 'end'
    ]

    # 32 tasks of mutual exclusive jobs cannot be executed on the same route (cause i need a different robot)
    mutual_exclusive = [
        Sum([direct_travel[k][i][j] for i, task_i in enumerate(tasks)]) == 0
        for k, vehicle in enumerate(vehicles)
        for j, task_j in enumerate(tasks)
        if vehicle.ATR_type not in task_j.ATR
    ]


    # 33 the operating range cannot exceed the given value
    domain = [
        autonomy_left[i][j] <= the_instance.Autonomy for i,_ in enumerate(vehicles)
                                                            for j,_ in enumerate(tasks)
    ]

    # 34 constraint over arrival time when precedence is required
    # i assume that precedence constraints can only be among tasks of the same job
    precedence = [
        customer_served[j1] <= customer_served[j2]
        for j1,task1 in enumerate(tasks)
        for j2,task2 in enumerate(tasks)
        if task1.id in task2.precedence
    ]

    # 35.1 set the arrival time within the time window for each customer
    time_window = [
        And(
            customer_served[j] >= task.TW[0],
            customer_served[j] >= task.TW[1]
        )
        for j,task in enumerate(tasks)
        if task.task_type != 'recharge'
        and task.TW != []
    ]

    # 36 based on the direct travels, define the arrival time to a customer depending on the departure time from the previous
    infer_arrival_time = [
        Implies(
        direct_travel[k][i][j],
        customer_served[j] >= customer_served[i] +
                                shortest_paths[(task_i.id,task_j.id)].length +
                                task_j.Service
        )
        for k,vehicle in enumerate(vehicles)
        for i,task_i in enumerate(tasks)
        for j,task_j in enumerate(tasks)
        if task_i.id != task_j.id
        and task_i.task_type != 'recharge'
    ]

    # 37 auxiliary constraint to connect charging time and state of charge
    charging_time_constraint = [
        charging_time[k][i]
        >=
        (1 / the_instance.charging_coefficient) * (the_instance.Autonomy - autonomy_left[k][i])
        for k,vehicle in enumerate(vehicles)
        for i,task in enumerate(tasks)
        if task.task_type == 'recharge'
    ]

    # 38 arrival time for charging stations
    infer_arrival_time_2 = [
        Implies(
            direct_travel[k][i][j],
            customer_served[j] >= customer_served[i] +
                                  shortest_paths[(task_i.id, task_j.id)].length +
                                  charging_time[k][i]
        )
        for k, vehicle in enumerate(vehicles)
        for i, task_i in enumerate(tasks)
        for j, task_j in enumerate(tasks)
        if task_i.id != task_j.id
        and task_i.task_type == 'recharge'
    ]

    # 39 The following two constraints model how energy is consumed by travelling and accumulated again by visiting the
    # charging stations
    autonomy = [
        Implies(
            direct_travel[k][i][j],
            autonomy_left[k][j] <= autonomy_left[k][i] - shortest_paths[(task_i.id,task_j.id)].length
        )
        for k, vehicle in enumerate(vehicles)
        for i, task_i in enumerate(tasks)
        for j, task_j in enumerate(tasks)
        if task_i.id != task_j.id
        and task_i.task_type != 'recharge'
    ]
    # 40
    autonomy_2 = [
        Implies(
            direct_travel[k][i][j],
            autonomy_left[k][j] <= the_instance.Autonomy - shortest_paths[(task_i.id, task_j.id)].length
        )
        for k, vehicle in enumerate(vehicles)
        for i, task_i in enumerate(tasks)
        for j, task_j in enumerate(tasks)
        if task_i.id != task_j.id
        and task_i.task_type == 'recharge'
    ]

    # 41 one arrival at each customer
    one_arrival = [
        Sum([direct_travel[k][i][j] for k,_ in enumerate(vehicles) for i,_ in enumerate(tasks)]) == 1
        for j,task_j in enumerate(tasks)
        if task_j.task_type != 'start'
        and task_j.task_type != 'end'
        and task_j.task_type != 'recharge'
    ]

    # 42 at most one arrival
    one_arrival_2 = [
        Sum([direct_travel[k][i][j] for k,_ in enumerate(vehicles) for i,_ in enumerate(tasks)]) <= 1
        for j,task_j in enumerate(tasks)
        if task_j.task_type == 'recharge'
    ]

    # 43 guarantee the flow conservation between start and end
    flow = [
        Sum([direct_travel[k][i][j] for j,task_j in enumerate(tasks) if task_j.task_type != 'start'])
        ==
        Sum([direct_travel[k][j][i] for j,task_j in enumerate(tasks) if task_j.task_type != 'end'])
        for k,_ in enumerate(vehicles)
        for i,task_i in enumerate(tasks)
        if task_i.task_type != 'start'
        and task_i.task_type != 'end'
    ]

    # 44 routes must be closed (i.e. every vehicle that goes out has to come back)
    route_continuity = [
        Sum([direct_travel[k][i][j] for j,_ in enumerate(tasks)])
        ==
        Sum([direct_travel[k][l][m] for l,_ in enumerate(tasks)])
        for k,_ in enumerate(vehicles)
        for i,task_i in enumerate(tasks)
        for m,task_m in enumerate(tasks)
        if task_i.task_type == 'start'
        and task_m.task_type == 'end'
        and task_i.location == task_m.location
    ]

   # 45 if a number of tasks belongs to one job, they have to take place in sequence
    mutual_exclusivity = [
        Sum([direct_travel[k][i][j] for k,_ in enumerate(vehicles)]) == 1
        for i,task_i in enumerate(tasks)
        for j,task_j in enumerate(tasks)
        if task_j.precedence != [] and task_i.id in task_j.precedence
    ]

    # 47 excluding previous routes ############ FIX THIS!!!!!!!!!!!! ##########
    if previous_routes != []:
        blabla = {index:i for index,i in enumerate(previous_routes)}
        excluding_previous_routes = [
            Sum([ direct_travel[k[0]][k[1]][k[2]] for k in blabla[j] ]) <= len(blabla[j]) - 1
            for j in blabla
        ]
        routing.add(excluding_previous_routes)

    # no travel from start to end (no empty routes)
    # NOT INCLUDED IN THE PAPER BECAUSE REDOUNDANT because of the cost function
    no_from_start_to_end = [
        Not(direct_travel[k][i][j])
        for k,_ in enumerate(vehicles)
        for i,task_i in enumerate(tasks)
        for j,task_j in enumerate(tasks)
        if task_i.task_type == 'start'
        and task_j.task_type == 'end'
    ]

    # vehicles can only start at the depot they are assigned to
    vehicles_to_their_depots = [
        Not(direct_travel[k][i][j])
        for k, vehicle in enumerate(vehicles)
        for i, task_i in enumerate(tasks)
        for j, task_j in enumerate(tasks)
        if vehicle.depot != task_i.location
        and task_i.task_type == 'start'
    ]

    # there can at most be one route for each vehicle
    one_vehicle_one_route = [
        Sum([direct_travel[k][i][j] for j, task_j in enumerate(tasks)]) <= 1
        for k, vehicle in enumerate(vehicles)
        for i, task_i in enumerate(tasks)
        if vehicle.depot == task_i.location
        and task_i.task_type == 'start'
    ]

    routing.add(
        opti +
        not_travel_same_spot +
        no_travel_to_start +
        no_travel_from_end +
        mutual_exclusive +
        domain +
        precedence +
        time_window +
        infer_arrival_time +
        infer_arrival_time_2 +
        charging_time_constraint +
        autonomy + autonomy_2 +
        one_arrival + one_arrival_2 +
        flow +
        route_continuity +
        mutual_exclusivity +
        no_from_start_to_end +
        vehicles_to_their_depots +
        one_vehicle_one_route

    )


    routes_plus = []
    # this list will be used to store the current solution for future runs of the solver
    current_solution = []
    if routing.check() != unsat:
        # print(routing.model())
        # print("TTD: ",m.getObjective().getValue())
        routing_feasibility = sat
        # route stores the info about each route that i generated with the VRPTW extended model
        routes = {}
        # segments stores the info o each direct travel from a task location to another
        segments = {}
        # here i populate the list based on the variable that evaluated to true in the model
        for k,vehicle in enumerate(vehicles):
            sub_segment = []
            for i,task_i in enumerate(tasks):
                for j,task_j in enumerate(tasks):
                    # print(k,i,j,m.getVarByName('direct_travel[{},{},{}]'.format(k,i,j)).X)
                    if routing.model()[direct_travel[k][i][j]] == True:
                        # print(direct_travel[k][i][j])
                        sub_segment.append((task_i,task_j))
                        current_solution.append([k, i, j])
            segments.update({vehicle:sub_segment})
        ########### IN CASE I WANNA TAKE A LOOK AT THE SOLUTION ####################
        # print('############ CUSTOMER SERVED ################')
        # for i in tasks:
        #     print(i,round(m.getVarByName('customer_served[{}]'.format(i)).X))
        # print('############# AUTONOMY LEFT #################')
        # for k in vehicles:
        #     for i in tasks:
        #         print(k,i,round(m.getVarByName('autonomy_left[{},{}]'.format(k,i)).X))
        ############################################################################

        # here i start forming the routes by adding the direct travels that begin from the start point
        for k in segments:
            for i in segments[k]:
                if i[0].task_type == 'start':
                    routes.update({k:[i]})
        # # here i concatenate the routes by matching the last location of a direct_travel with the first of another one
        for j in routes:
            while routes[j][-1][1].task_type != 'end':
                for i in segments[j]:
                    if i[0].id == routes[j][-1][1].id:
                        routes[j].append(i)
        # let's just change the format of storing the routes
        routes_2 = {route:[routes[route][0][0]] for route in routes}
        for route1, route2 in zip(routes, routes_2):
            for segment in routes[route1]:
                routes_2[route2].append(segment[1])
        # assign routes_2 to routes and keep using routes
        routes = routes_2

        # this list tells the distance between each two locations based on the route
        # first step: calculate distance from one location to the following
        routes_length = {
            route:sum([
                    shortest_paths[(elem1.id, elem2.id)].length
                    for elem1, elem2 in zip(routes[route][:-1], routes[route][1:])
                ])
            for route in routes
        }

        # this list tells me the actual nodes that the vehicles must visit to execute the route and if
        # the node has a time window ([] otherwise)
        actual_nodes = {}
        for r,route in enumerate(routes):
            points = [routes[route][0].location]
            tws = [[]]
            St = [0]
            Ct = [0]
            for s,(segment1, segment2) in enumerate(zip(routes[route][:-1], routes[route][1:])):
                points += shortest_paths[(segment1.id,segment2.id)].path_nodes[1:]
                for _ in shortest_paths[(segment1.id,segment2.id)].path_nodes[1:]:
                    tws.append([])
                    St.append(0)
                    Ct.append(0)
                tws[-1] = segment2.TW
                St[-1] =segment2.Service
                if segment2.task_type == 'recharge':
                    Ct[-1] = math.ceil((1 / the_instance.charging_coefficient) \
                    *\
                    (the_instance.Autonomy
                     - round(float(str(routing.model()[autonomy_left[r][s]])))))

            actual_nodes.update({route:(points, tws, St,Ct)})

        actual_nodes_2 = {
            elem:[(current, next) for current, next in zip(
                actual_nodes[elem][0],
                actual_nodes[elem][0][1:])
             ]
            for elem in actual_nodes
        }

        for route in routes:
            routes_plus.append(
                Route(
                    routes[route],
                    route,
                    routes_length[route],
                    actual_nodes[route][0],
                    actual_nodes_2[route],
                    actual_nodes[route][1],
                    actual_nodes[route][2],
                    actual_nodes[route][3]
                )
            )
    else:
        routing_feasibility = unsat

    return routing_feasibility, routes_plus, current_solution




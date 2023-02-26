import math

cities = [['A',[0, 0]],
          ['B',[30, 3]],
          ['C',[60, 4]],
          ['D',[2, 30]],
          ['E',[30, 30]],
          ['F',[5, 39]]]

def tsgCreateRoute(missionList):
    route = []
    for cmd in missionList:
        route.append([[cmd.x, cmd.y], cmd])
    return route

def tspGetDistance(route):
    dist = 0
    for i in range(len(route)-1):
        segDist = math.sqrt((route[i][1][0] - route[i+1][1][0])**2 + (route[i][1][1] - route[i+1][1][1])**2)
        dist += segDist
    return dist
    
print('Current route')
print(cities)
print(tspGetDistance(cities))

def tspSingle2opt(route):
    best_distance = tspGetDistance(route)
    first = 1
    last = len(route)
    for i in range(last-2):
        for j in range(last-3-i):
            j += 2 + i
            start = route[0:first+i]
            middle = route[first+i:j+1]
            end = route[j+1:last]
            new_route = start + middle[::-1] + end
            new_distance = tspGetDistance(new_route)
            if new_distance < best_distance:
                best_distance = new_distance
                route = new_route
    return route

def tsp2opt(route):
    currentDistance = tspGetDistance(route)
    newRoute = tspSingle2opt(route)
    newDistance = tspGetDistance(newRoute)
    
    while newDistance / currentDistance < 1:
        route = newRoute
        currentDistance = newDistance
        newRoute = tspSingle2opt(route)
        newDistance = tspGetDistance(newRoute)
    
    return route


cities = tsp2opt(cities)
print('New route')
print(cities)
print(tspGetDistance(cities))
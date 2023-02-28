import math
import haversine as hs

def tsgCreateRoute(missionList):
    route = []
    for cmd in missionList:
        route.append([[cmd.x, cmd.y], cmd])
    return route

def tspGetDistanceHS(route):
    dist = 0
    for i in range(len(route)-1):
        segDist = hs.haversine((route[i][0][0], route[i][0][1]), (route[i+1][0][0], route[i+1][0][1]))
        dist += segDist
    return dist

def tspSingle2opt(route):
    best_distance = tspGetDistanceHS(route)
    first = 1
    last = len(route)
    for i in range(last-2):
        for j in range(last-3-i):
            j += 2 + i
            start = route[0:first+i]
            middle = route[first+i:j+1]
            end = route[j+1:last]
            new_route = start + middle[::-1] + end
            new_distance = tspGetDistanceHS(new_route)
            if new_distance < best_distance:
                best_distance = new_distance
                route = new_route
    return route

def tsp2opt(missionList):
    route = tsgCreateRoute(missionList)
    currentDistance = tspGetDistanceHS(route)
    newRoute = tspSingle2opt(route)
    newDistance = tspGetDistanceHS(newRoute)
    
    while newDistance / currentDistance < 1:
        route = newRoute
        currentDistance = newDistance
        newRoute = tspSingle2opt(route)
        newDistance = tspGetDistanceHS(newRoute)

    missionList = []
    for point in newRoute:
        missionList.append(point[1])
    
    return missionList

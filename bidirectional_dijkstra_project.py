import math
import heapq
def ShortestPath(s,dist,prev,proc,t,rev_dist,rev_prev,rev_proc):
    distance = math.inf
    #u_best = None
    for u in proc + rev_proc:
        if dist[u]+rev_dist[u]<distance:
            #u_best = u
            distance = dist[u]+rev_dist[u]
    """
    path = []
    last = u_best
    while last!= u_best:
        path.append(last)
        last = prev[last]
    path = path.reverse()
    last = u_best
    while last!=t:
        last = rev_prev[last]
        path.append(last)
    """
    return distance
def relax(u,v,dist,prev,cost,h):
    if dist[v] > dist[u] + cost:
        dist[v] = dist[u]+cost
        prev[v] = prev[u]
        heapq.heappush(h, [dist[v],v])


def process(u,G,cost,dist,prev,proc,h):

    for i,ele in enumerate(G[u]):
        relax(u,ele,dist,prev,cost[u][i],h)
    proc.append(u)

def BidirectionaDijkstra(gra,rev_gra,cost_1,cost_2,s,t):
    dist = [math.inf]*n
    rev_dist = [math.inf]*n
    dist[s]=0
    hq = []
    heapq.heappush(hq,[dist[s],s])
    rev_hq = []
    heapq.heappush(rev_hq,[dist[t],t])
    rev_dist[t]=0
    prev = [None]*n
    rev_prev = [None]*n
    proc = []
    rev_proc = []
    while True:
        try:
            dis,v = heapq.heappop(hq)
            process(v,gra,cost_1,dist,prev,proc,hq)
            if v in rev_proc:
                return ShortestPath(s,dist,prev,proc,t,rev_dist,rev_prev,rev_proc)
            rev_dis,rev_v = heapq.heappop(rev_hq)
            process(rev_v,rev_gra,cost_2,rev_dist,rev_prev,rev_proc,rev_hq)
            if v in rev_proc:
                return ShortestPath(s,dist,prev,proc,t,rev_dist,rev_prev,rev_proc)
        except:
            return -1


if __name__ == '__main__':
    n, m = map(int,input().split())
    adj = [[[] for _ in range(n)], [[] for _ in range(n)]]
    cost = [[[] for _ in range(n)], [[] for _ in range(n)]]
    for e in range(m):
        u, v, c = map(int,input().split())
        adj[0][u - 1].append(v - 1)
        cost[0][u - 1].append(c)
        adj[1][v - 1].append(u - 1)
        cost[1][v - 1].append(c)
    #t = int(input())
    s,t = map(int,input().split())
    print(BidirectionaDijkstra(adj[0],adj[1],cost[0],cost[1],s-1,t-1))

    #for i in range(t):
        #s, t = map(int,input().split())
        #print(BidirectionaDijkstra(adj[0],adj[1],cost[0],cost[1],s-1,t-1))
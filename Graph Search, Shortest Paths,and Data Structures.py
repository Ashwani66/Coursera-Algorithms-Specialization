#PA 1 :scc_finder.py
class SccFinder(object):
    def __init__(self, input_file):
        self.scc_list = []
        with open(input_file) as file:
            self.finish_order = []
            self._graph = {}
            for line in file:
                (from_v, to_v) = tuple(number for number in line.split())
                self._add_edge_to_graph(int(from_v), int(to_v))

    def _add_edge_to_graph(self, from_v, to_v):
        if from_v in self._graph:
            self._graph[from_v].append(to_v)
        else:
            self._graph[from_v] = [to_v]
        if to_v in self._graph:
            self._graph[to_v].append(-from_v)
        else:
            self._graph[to_v] = [-from_v]

    def compute_finish_times(self):
        visited_nodes, finished_nodes = set(), set()
        for vertex in self._graph.keys():
            if vertex in visited_nodes:
                continue
            nodes_stack = [vertex]
            while nodes_stack:
                node = nodes_stack.pop()
                if node not in visited_nodes:
                    visited_nodes.add(node)
                    nodes_stack.append(node)
                    neighbors = (-edge for edge in self._graph[node] if edge < 0)
                    for neighbor in neighbors:
                        if neighbor not in visited_nodes:
                            nodes_stack.append(neighbor)
                else:
                    if node not in finished_nodes:
                        self.finish_order.append(node)
                        finished_nodes.add(node)

    def compute_sccs(self):
        visited_nodes = set()
        assert (len(self.finish_order) == len(self._graph))
        for i in reversed(self.finish_order):
            if i in visited_nodes:
                continue
            nodes_stack = [i]
            size = 0
            while nodes_stack:
                node = nodes_stack.pop()
                if node not in visited_nodes:
                    size += 1
                    visited_nodes.add(node)
                    nodes_stack.append(node)
                    neighbors = (edge for edge in self._graph[node] if edge > 0)
                    for neighbor in neighbors:
                        if neighbor not in visited_nodes:
                            nodes_stack.append(neighbor)
            self.scc_list.append(size)
        self.scc_list.sort(reverse=True)
        print(self.scc_list[:5])

if __name__ == "__main__":
    scc_finder = SccFinder("assignment_4.txt")
    scc_finder.compute_finish_times()
    scc_finder.compute_sccs()
    expected_sccs = [434821, 968, 459, 313, 211]
    print(scc_finder.scc_list[:5])

#Output: [434821, 968, 459, 313, 211]



#PA 2:dijkstra_path_finder.py

from ast import literal_eval


class DijkstraPathFinder:

    def __init__(self, input_file):
        self.graph = {}
        with open(input_file) as file:
            for line in file:
                line_content = line.split()
                self.graph[int(line_content[0])] = [literal_eval(edge) for edge in line_content[1:]]
        self._source_vertex = next(iter(self.graph.keys()))

    def compute_shortest_paths(self, source=None):
        if source is None:
            source = self._source_vertex
        shortest_paths = {}
        visited = set()
        for vertex in self.graph.keys():
            shortest_paths[vertex] = (9999999999, [])
        shortest_paths[source] = (0, [])
        visited.add(source)
        while set(self.graph.keys() - visited):
            source, min_edge = -1, ()
            for vertex in visited:
                for edge in self.graph[vertex]:
                    if edge[0] in visited:
                        continue
                    if not min_edge or shortest_paths[vertex][0] + edge[1] < min_edge[1]:
                        min_edge = (edge[0], shortest_paths[vertex][0] + edge[1])
                        source = vertex
            shortest_paths[min_edge[0]] = (min_edge[1], shortest_paths[source][1] + [min_edge[0]])
            visited.add(min_edge[0])
        return shortest_paths

if __name__ == '__main__':
    path_finder = DijkstraPathFinder('assignment_5.txt')
    paths = path_finder.compute_shortest_paths()
    actual = {vertex: distance[0] for (vertex, distance) in paths.items()}
    print(actual[7])
    print(actual[37])
    print(actual[59])
    print(actual[82])
    print(actual[99])
    print(actual[115])
    print(actual[133])
    print(actual[165])
    print(actual[188])
    print(actual[197])

'''Output 2599
2610
2947
2052
2367
2399
2029
2442
2505
30688
'''

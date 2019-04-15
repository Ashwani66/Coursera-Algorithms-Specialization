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
3068
'''

#PA 3: median_maintainer.py

import heapq


class Heap(object):
    def __init__(self, initial=None, key=lambda x: x):
        self.key = key
        if initial:
            self._data = [(key(item), item) for item in initial]
            heapq.heapify(self._data)
        else:
            self._data = []

    def push(self, item):
        heapq.heappush(self._data, (self.key(item), item))

    def pop(self):
        return heapq.heappop(self._data)[1]

    def peek(self):
        return self._data[0][1]

    def __len__(self):
        return len(self._data)


class MedianMaintainer:
    def __init__(self, input_file=None, input_array=None):
        self._heap_low = Heap(key=lambda x: -x)
        self._heap_high = Heap()
        self._median_sum = 0
        self.input_file = input_file
        self.input_array = input_array

    def sum_medians(self):
        if self.input_file is not None:
            with open(self.input_file) as file:
                for number in file.read().splitlines():
                    self._add_number(int(number))
        elif self.input_array is not None:
            for number in self.input_array:
                self._add_number(int(number))
        return self._median_sum % (len(self._heap_high) + len(self._heap_low))

    def _add_number(self, num):
        if not self._heap_low:
            self._heap_low.push(num)
            self._median_sum += num
            return
        if num <= self._heap_low.peek():
            self._heap_low.push(num)
        else:
            self._heap_high.push(num)
        if len(self._heap_low) - len(self._heap_high) > 1:
            self._heap_high.push(self._heap_low.pop())
        elif len(self._heap_high) - len(self._heap_low) > 1:
            self._heap_low.push(self._heap_high.pop())
        self._median_sum += self._heap_low.peek() if len(self._heap_low) >= len(
            self._heap_high) else self._heap_high.peek()

if __name__ == "__main__":
    median_maintainer = MedianMaintainer(input_file='assignment_3.txt')
    median_sum = median_maintainer.sum_medians()
    print(median_sum)

#Output : 1213


# PA:4 two_sum_finder.py

from bisect import bisect_left, bisect_right


class TwoSumFinder:
    def __init__(self, input_file=None):
        self._array = []
        numbers = set()
        self._target_values = 0
        if input_file is None:
            for number in input().split():
                numbers.add(int(number))
        else:
            with open(input_file) as file:
                for number in file.read().splitlines():
                    numbers.add(int(number))
        self._array = sorted(numbers)

    def compute_values(self):
        target_values = set()
        for num in self._array:
            low = bisect_left(self._array, -10000 - num)
            high = bisect_right(self._array, 10000 - num)
            for pair_num in self._array[low:high]:
                if pair_num != num:
                    target_values.add(num + pair_num)
        return len(target_values)

if __name__ == "__main__":
    two_sum_finder = TwoSumFinder("assignment_4.txt")
    target_values = two_sum_finder.compute_values()
    print(target_values)

#Output:  427

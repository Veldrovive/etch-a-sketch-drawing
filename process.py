from svgpathtools import svg2paths, Path
import numpy as np
from matplotlib import pyplot as plt
from typing import List, Tuple, Dict, Any, Optional
from sklearn.cluster import MeanShift
from sklearn.neighbors import KDTree
import imageio

from pathlib import Path as DiskPath

import networkx as nx
import networkx.algorithms.approximation as nx_app

from intersect_lib import isect_segments_include_segments

# Using this info, we can calculate the step resolution of our space as
WIDTH_TURNS = 4
HEIGHT_TURNS = 2.5
GEAR_RATIO = 2
STEPS_PER_REV = 200
MICROSTEPS = 1
WIDTH_RESOLUTION = (STEPS_PER_REV * MICROSTEPS) * GEAR_RATIO * WIDTH_TURNS
HEIGHT_RESOLUTION = (STEPS_PER_REV * MICROSTEPS) * GEAR_RATIO * HEIGHT_TURNS
PADDING_RATIO = 0.05
DPR = 0.1  # Dots per resolution. Should be less than 1

def get_continuous_paths(paths: List[Path]):
    """
    Takes a list of paths and splits them all up into their continuous subpaths
    """
    continuous_paths = []
    for path in paths:
        continuous_subpaths = path.continuous_subpaths()
        continuous_paths.extend(continuous_subpaths)
    return continuous_paths

def sample_paths(paths: List[Path], screen_width: int, screen_height: int, dpr: float=0.1):
    """
    Samples the paths at many points
    """
    # Get the bounding box of the entire set of paths
    min_x = np.inf
    min_y = np.inf
    max_x = -np.inf
    max_y = -np.inf
    for p in paths:
        # print(p)
        try:
            p_min_x, p_max_x, p_min_y, p_max_y = p.bbox()
            min_x = min(min_x, p_min_x)
            min_y = min(min_y, p_min_y)
            max_x = max(max_x, p_max_x)
            max_y = max(max_y, p_max_y)
        except ValueError:
            # This is a degenerate path, so we can just skip it
            pass
    width, height = max_x - min_x, max_y - min_y
    scale = min(screen_width / width, screen_height / height)

    sampled_paths = []
    for path in paths:
        length = path.length()
        # num_points = max(int(length / width * screen_width * dpr), int(length / height * screen_height * dpr))
        num_points = int(length * scale * dpr)
        
        sampled_path = []
        sampled_ts = []
        if num_points < 2:
            # This is too small to sample
            zero_point = path.point(0)
            sampled_path.append(zero_point)
            sampled_ts.append(0)
            one_point = path.point(1)
            if zero_point != one_point:
                sampled_path.append(one_point)
                sampled_ts.append(1)
            continue
        for i in range(num_points + 1):
            prop = i / num_points
            s = length * prop
            t = path.ilength(s)
            # point = (path.point(t) - zero_offset) * scale + center_offset
            point = path.point(t)
            sampled_path.append(point)
            sampled_ts.append(t)
        sampled_paths.append((path, sampled_path, sampled_ts))
    return sampled_paths

def get_intersections(lines: List[Tuple[Path, List[complex], List[float]]]):
    """
    Finds all intersections between the lines
    When computing intersections we also add "fake" segments that are the start and end points of the lines to make sure we get endpoint intersections with low quality svgs

    """
    # Lines is a list of complex numbers and we need to convert it into a list of segments where are tuples of tuples
    segments = []
    segment_map: Dict[Tuple[Tuple[float, float], Tuple[float, float]], int] = {}
    for line in lines:
        samples = line[1]
        # Add in fake start and end points to extend the lines in their current direction
        fake_start = samples[0] - (samples[1] - samples[0])
        fake_end = samples[-1] + (samples[-1] - samples[-2])
        samples = [fake_start] + samples + [fake_end]
        for i in range(len(samples) - 1):
            v1 = (samples[i].real, samples[i].imag)
            v2 = (samples[i + 1].real, samples[i + 1].imag)
            if v1 > v2:
                v1, v2 = v2, v1
            segment = (v1, v2)
            segments.append(segment)
            segment_map[segment] = lines.index(line)
    res = isect_segments_include_segments(segments)
    # res is a list of (point, List[segment])
    # To process these, we want convert this into (point, List[segment], List[(i, path)])
    intersections = []
    for intersection in res:
        point = intersection[0]
        segments = intersection[1]
        segment_indices = []
        for segment in segments:
            segment_indices.append(segment_map[segment])
        intersections.append((point, segments, segment_indices))
    # TODO: We are not looking for intersections between adjacent segments on the same path so we need to filter these out
    # For now we just filter out all self-intersections. That is, intersections where all segments belong to the same path
    filtered_intersections = []
    removed_intersections = []
    for point, segments, segment_paths in intersections:
        unique_paths = set(segment_paths)
        if len(unique_paths) > 1:
            filtered_intersections.append((point, segments, segment_paths))
        else:
            removed_intersections.append((point, segments, segment_paths))

    return intersections, filtered_intersections, removed_intersections

def find_intersection_parameterization(lines: List[Tuple[Path, List[complex], List[float]]], intersections: List[Tuple[Tuple[float, float], List[Tuple[Tuple[float, float], Tuple[float, float]]], List[int]]]):
    """
    Converts the intersection points into List[Tuple[Tuple[float, float], List[Tuple[path, float]]]]
    This is the point of intersection mapped to the intersecting paths and the t values they are intersecting at.
    We can compute this by taking the closest sample on the path to the intersection point

    Parameters - 
        lines: A list of tuples of the form (path, List[samples], List[t values])
        intersections: A list of tuples of the form (point, List[segments], List[path indices])
    """
    intersection_parameterization = []
    for intersection in intersections:
        point = complex(*intersection[0])
        intersection_paths = intersection[2]

        parameterized_intersections = []
        for path_index in intersection_paths:
            path = lines[path_index][0]
            samples = lines[path_index][1]
            t_values = lines[path_index][2]
            # Find the closest sample to the intersection point
            closest_sample = min(samples, key=lambda sample: abs(sample - point))
            closest_sample_index = samples.index(closest_sample)
            closest_t = t_values[closest_sample_index]
            parameterized_intersections.append((path, closest_t))
        intersection_parameterization.append((point, parameterized_intersections))
    return intersection_parameterization

def mean_shift_cluster(data: List[Tuple[complex, List[Tuple[complex, List[Tuple[Path, float]]]]]]) -> List[Tuple[Tuple[float, float], List[Tuple[complex, List[Tuple[Path, float]]]]]]:
    """
    De-dupes intersections that were found multiple times
    """
    # Convert the data to a numpy array of points
    points = np.array([(p[0].real, p[0].imag) for p in data])  ## TODO: This is not a way to get width and height
    width = max(points[:, 0]) - min(points[:, 0])
    height = max(points[:, 1]) - min(points[:, 1])

    band_width = min(width, height) / 100

    ms = MeanShift(bandwidth=band_width)
    ms.fit(points)
    labels = ms.labels_
    cluster_centers = ms.cluster_centers_

    # Create a dictionary to store the concatenated metadata for each cluster
    metadata_dict = {}

    # Loop through each point and its label
    for point, label, metadata in zip(points, labels, data):
        # If the label is not in the dictionary yet, create an empty list for it
        if label not in metadata_dict:
            metadata_dict[label] = set()
        
        # Append the metadata of this point to the list of its cluster label
        metadata_dict[label].update(metadata[1])
    
    # Create an empty list to store the output tuples
    output = []

    # Loop through each cluster label and its center
    for label, center in enumerate(cluster_centers):
        # Create a tuple of the cluster center and its concatenated metadata
        output_tuple = (tuple(center), list(metadata_dict[label]))

        # Append the tuple to the output list
        output.append(output_tuple)
    
    # Return the output list
    return output

def build_intersection_graph(lines: List[Tuple[Path, List[complex], List[float]]], intersections: List[Tuple[complex, List[Tuple[Path, float]]]]):
    """
    Builds a graph where nodes are intersection points (Tuple[float, float]) and edges are the path subsection that joins them (Tuple[Path, min_t, max_t])
    To build this, we will first invert the intersections list into a map of path -> List[(t, intersection point)] where the list is sorted by increasing t
    We can then, for each node, start with t=0, check if there is already a node at this point, and if not, add it
    Then we can iterate over the list of intersections and add nodes and edges as we go along
    When we have reached the end of the intersections if the last t value is less than 1, we need to add a final node and edge
    """
    # Build the path -> List[(t, intersection point)] map
    path_map: Dict[Path, List[Tuple[float, complex]]] = {}
    # Since some paths may not intersect, we initialize by adding all paths to the map
    for line in lines:
        path_map[line[0]] = []

    for intersection in intersections:
        point = complex(*intersection[0])
        for path, t in intersection[1]:
            path_map[path].append((t, point))
    
    # Sort the list of intersections by increasing t
    for path in path_map:
        path_map[path].sort(key=lambda x: x[0])
    
    # Create the graph. For this we use an adjacency list representation since we expect the graph to be very sparse
    graph = {}
    def add_edge(node1, node2, path, t_start, t_end):
        """
        Adds the edge to the graph.
        If a node does not exist yet, it is created
        The graph is undirected so we add an edge for both nodes
        """
        t_min, t_max = min(t_start, t_end), max(t_start, t_end)
        if node1 not in graph:
            graph[node1] = []
        if node2 not in graph:
            graph[node2] = []
        graph[node1].append((node2, (path, t_min, t_max)))
        graph[node2].append((node1, (path, t_min, t_max)))

    for path in path_map:
        intersections = path_map[path]
        # If the first t value is not 0, we need to add a node at the beginning of the path
        if len(intersections) < 1 or intersections[0][0] > 0:
            intersections = [(0, path.point(0))] + intersections
        # If the last t value is not 1, we need to add a node at the end of the path
        if len(intersections) < 1 or intersections[-1][0] < 1:
            intersections = intersections + [(1, path.point(1))]
        # Iterate over the intersections and add nodes and edges
        for i in range(len(intersections) - 1):
            t1, point1 = intersections[i]
            t2, point2 = intersections[i + 1]
            add_edge(point1, point2, path, t1, t2)

    return graph

def join_graph_components(graph: Dict[complex, List[Tuple[complex, Tuple[Path, float, float]]]], lines: List[Tuple[Path, List[complex], List[float]]]):
    """
    Finds the connected components of the graph and joins them by adding a set of edges with minimum weight that connect them
    This is a multistep process:
    1. Find the connected components of the graph
    2. For each component pair, find the minimum distance points between them
    3. Solve the TSP using each component as a node
    4. Add the edges that result in a minimum tour of the connected components
    """
    # In order to do this more easily, we will use the package networkx
    intersection_graph = nx.MultiGraph()
    # Each node will be the complex number and each edge will have a `weight`, `path`, `t_min`, and `t_max` attribute
    for node in graph:
        intersection_graph.add_node(node)

    for node1 in graph:
        for node2, edge in graph[node1]:
            intersection_graph.add_edge(node1, node2, weight=abs(node1 - node2), path=edge[0], t_min=edge[1], t_max=edge[2], key=(edge[0], edge[1], edge[2]))
    
    # Find the connected components of the graph
    components = list(nx.connected_components(intersection_graph))
    if len(components) == 1:
        return intersection_graph
    # In order to find the closest distance, we need to create kd trees of samples for each component
    path_to_sample_list = {}
    for line in lines:
        path = line[0]
        samples = line[1]
        t_values = line[2]
        path_to_sample_list[path] = (samples, t_values)
    
    sample_points = {}
    point_to_t_map = {}
    point_to_edge_map = {}
    for i in range(len(components)):
        sample_points[i] = []
        subgraph = intersection_graph.subgraph(components[i])
        for edge in subgraph.edges:
            # Get the path and the t values
            path = subgraph.edges[edge]["path"]
            t_min = subgraph.edges[edge]["t_min"]
            t_max = subgraph.edges[edge]["t_max"]
            samples = path_to_sample_list[path]
            edge_samples = [sample for sample, t in zip(samples[0], samples[1]) if t_min <= t <= t_max]
            t_samples = [t for sample, t in zip(samples[0], samples[1]) if t_min <= t <= t_max]
            for sample, t in zip(edge_samples, t_samples):
                point_to_edge_map[sample] = edge
                point_to_t_map[sample] = t
            sample_points[i] += edge_samples
    # Create the kd trees
    component_kd_trees = {}
    for i in range(len(components)):
        points = np.array([np.array([point.real, point.imag]) for point in sample_points[i]])
        component_kd_trees[i] = KDTree(points)
    
    # Now for each pair of components we find the closest distance between them
    component_graph = nx.Graph()
    for i in range(len(components)):
        component_graph.add_node(i)

    for i in range(len(components)):
        for j in range(i + 1, len(components)):
            if len(sample_points[i]) < len(sample_points[j]):
                # Then we want to use i as the query and j as the data
                query_component = i
                data_component = j
            else:
                query_component = j
                data_component = i
            # Find the closest distance between the two components by iterative over all point in the query component and finding the closest point in the data component
            closest_distance = float("inf")
            closest_query_edge = None
            closest_data_edge = None
            for query_point in sample_points[query_component]:
                # Find the closest point in the data component
                query_arr = np.array([query_point.real, query_point.imag])
                dist, ind = component_kd_trees[data_component].query([query_arr])
                dist = dist[0][0]
                ind = ind[0][0]
                if dist < closest_distance:
                    closest_distance = dist
                    closest_query_point = query_point
                    closest_data_point = sample_points[data_component][ind]
                    closest_query_edge = (closest_query_point, point_to_edge_map[closest_query_point], point_to_t_map[closest_query_point])
                    closest_data_edge = (sample_points[data_component][ind], point_to_edge_map[closest_data_point], point_to_t_map[closest_data_point])
            # Add the edge to the graph
            component_graph.add_edge(i, j, weight=closest_distance, query_edge=closest_query_edge, data_edge=closest_data_edge)

    # Now to find which edges to actually add to the graph, we will solve the TSP
    cycle = nx_app.traveling_salesman_problem(component_graph, method=nx_app.christofides)

    # To do this, we will first iterate over collecting information on which edges are split and where
    # In a second pass we will sort by the t values where splitting is happening and then actually split the edges
    # In the case where the t value for two splits is the same on an edge

    # We need two data structures. One to keep track of which new nodes will be added at what t values on which edges
    # And another to keep track of what edges will be added once the nodes have been put in place
    edges_to_split: Dict[Any, Tuple[Path, Tuple[complex, float], Tuple[complex, float], List[Tuple[complex, float]]]] = {}  # Maps from the edge used to index intersection_graph.edges to the t=0 node and t=1 node and a list of (new node point, t) tuples
    new_edges: List[Tuple[complex, complex]] = []  # List of new nodes to connect
    # First, we build up these data structures
    for u, v in nx.utils.pairwise(cycle[:-1]):
        query_edge_data = component_graph.edges[u, v]["query_edge"]
        data_edge_data = component_graph.edges[u, v]["data_edge"]
        query_node_point = query_edge_data[0]
        query_edge = query_edge_data[1]  # [0] is the first node, [1] is the second nodes, and [2] is the edge key
        query_t = query_edge_data[2]
        data_node_point = data_edge_data[0]
        data_edge = data_edge_data[1]
        data_t = data_edge_data[2]

        # QUERY EDGE:
        orig_edge = intersection_graph.edges[query_edge]
        edge_nodes = [query_edge[0], query_edge[1]]
        orig_t_min = orig_edge["t_min"]
        orig_t_max = orig_edge["t_max"]
        orig_path = orig_edge["path"]
        orig_min_point = orig_path.point(orig_t_min)
        orig_min_node = edge_nodes[0] if abs(orig_min_point - edge_nodes[0]) < abs(orig_min_point - edge_nodes[1]) else edge_nodes[1]
        orig_max_point = orig_path.point(orig_t_max)
        orig_max_node = edge_nodes[0] if abs(orig_max_point - edge_nodes[0]) < abs(orig_max_point - edge_nodes[1]) else edge_nodes[1]
        assert orig_min_node in edge_nodes
        assert orig_max_node in edge_nodes
        if query_edge not in edges_to_split:
            edges_to_split[query_edge] = (orig_path, (orig_min_node, orig_t_min), (orig_max_node, orig_t_max), [])
        edges_to_split[query_edge][3].append((query_node_point, query_t))

        # DATA EDGE:
        orig_edge = intersection_graph.edges[data_edge]
        edge_nodes = [data_edge[0], data_edge[1]]
        orig_t_min = orig_edge["t_min"]
        orig_t_max = orig_edge["t_max"]
        orig_path = orig_edge["path"]
        orig_min_point = orig_path.point(orig_t_min)
        orig_min_node = edge_nodes[0] if abs(orig_min_point - edge_nodes[0]) < abs(orig_min_point - edge_nodes[1]) else edge_nodes[1]
        orig_max_point = orig_path.point(orig_t_max)
        orig_max_node = edge_nodes[0] if abs(orig_max_point - edge_nodes[0]) < abs(orig_max_point - edge_nodes[1]) else edge_nodes[1]
        assert orig_min_node in edge_nodes
        assert orig_max_node in edge_nodes
        if data_edge not in edges_to_split:
            edges_to_split[data_edge] = (orig_path, (orig_min_node, orig_t_min), (orig_max_node, orig_t_max), [])
        edges_to_split[data_edge][3].append((data_node_point, data_t))

        # Add the new edge between the two nodes
        new_edges.append((query_node_point, data_node_point))

    # Now we actually split the edges
    for edge, (path, orig_min_node_data, orig_max_node_data, split_points) in edges_to_split.items():
        split_points.sort(key=lambda x: x[1])
        orig_min_node, orig_t_min = orig_min_node_data
        orig_max_node, orig_t_max = orig_max_node_data

        # Remove the original edge
        key = (path, orig_t_min, orig_t_max)
        intersection_graph.remove_edge(edge[0], edge[1], key=edge[2])

        # Iterate over t values and string together the new edges
        node_string = [orig_min_node_data] + split_points + [orig_max_node_data]
        for i in range(len(node_string) - 1):
            node1 = node_string[i][0]
            node2 = node_string[i + 1][0]
            if node2 not in intersection_graph:
                intersection_graph.add_node(node2)
            t1 = node_string[i][1]
            t2 = node_string[i + 1][1]
            if t1 == t2:
                # Then we have already added this edge because it is just a single node
                continue
            intersection_graph.add_edge(node1, node2, weight=abs(node1 - node2), path=path, t_min=t1, t_max=t2, key=(path, t1, t2))

    # Add the new edges
    for u, v in new_edges:
        assert u in intersection_graph
        assert v in intersection_graph
        intersection_graph.add_edge(u, v, weight=abs(u - v), path=None, t_min=None, t_max=None, key=(None, None, None))

    new_connected_components = list(nx.connected_components(intersection_graph))
    if len(new_connected_components) > 1:
        print("ERROR: Intersection graph is not connected!")
        # Find the largest component
        largest_component = max(new_connected_components, key=len)
        intersection_graph = intersection_graph.subgraph(largest_component)
    # assert len(new_connected_components) == 1

    return intersection_graph

def compute_tour(intersection_graph: nx.Graph, lines: List[Tuple[Path, List[complex], List[float]]]):
    """
    We assume that intersection_graph is connected.
    Computes a postman's tour of the graph and returns samples along the tour
    """
    path_to_sample_list = {}
    for line in lines:
        path = line[0]
        samples = line[1]
        t_values = line[2]
        path_to_sample_list[path] = (samples, t_values)

    """
    or this?
    cpp = nx.algorithms.euler.ChinesePostman(G)
    cpp_solution = cpp.find_optimal()
    """
    H = nx.eulerize(intersection_graph)

    samples = []
    for u, v, key in nx.eulerian_path(H, keys=True):
        edge = H.get_edge_data(u, v, key)
        if "path" in edge and edge["path"] is None:
            # Then this edge joins two components and we can just add the start and end points
            # samples.append(u)  # We don't actually need these because the next edge will have the same start point
            # samples.append(v)
            continue

        if "path" not in edge:
            # This edge was added by the eulerize function
            potential_edges = intersection_graph.get_edge_data(u, v)
            # This is a dictionary of keys to edges. We will use the one that has the least weight
            min_weight_metadata = sorted(list(potential_edges.values()), key=lambda x: x["weight"])[0]
            path = min_weight_metadata["path"]
            t_min = min_weight_metadata["t_min"]
            t_max = min_weight_metadata["t_max"]
            if path is None:
                # Then the eulerize added an edge that joins two components so we can just skip this one
                continue
        else:
            path = edge["path"]
            t_min = edge["t_min"]
            t_max = edge["t_max"]
        edge_samples = [sample for sample, t in zip(path_to_sample_list[path][0], path_to_sample_list[path][1]) if t_min <= t <= t_max]
        if len(edge_samples) > 0:
            # Check which end of the edge is closer to u. If it is the end of the edge, then we need to reverse the edge samples
            if abs(edge_samples[0] - u) > abs(edge_samples[-1] - u):
                edge_samples = reversed(edge_samples)
            samples.extend(edge_samples)
            # TODO: There are a bunch of duplicate control points because the start and end of adjacent edges are the same. Either remove these or don't add them in the first place

    # Samples is now our raw control inputs
    return samples

def process_control_inputs(samples: List[complex]) -> List[complex]:
    """
    Computes a list of control inputs that are in the control space of the etch-a-sketch
    The drawing should take up as much of the WIDTH_RESOLUTION x HEIGHT_RESOLUTION space as possible so it needs to be centered and expanded
    """
    width = WIDTH_RESOLUTION - 2*PADDING_RATIO*WIDTH_RESOLUTION
    height = HEIGHT_RESOLUTION - 2*PADDING_RATIO*HEIGHT_RESOLUTION
    # First, we expand the drawing to fill an entire width x height space and then center it
    s_minx, s_miny, s_maxx, s_maxy = min(samples, key=lambda x: x.real).real, min(samples, key=lambda x: x.imag).imag, max(samples, key=lambda x: x.real).real, max(samples, key=lambda x: x.imag).imag
    s_width = s_maxx - s_minx
    s_height = s_maxy - s_miny
    
    # Compute the scale factor
    scale_factor = min(width/s_width, height/s_height)
    scaled_samples = [scale_factor * sample for sample in samples]

    # Now that the drawing fits within a width x height space, we can just subtract the minx and miny to center it
    s_minx, s_miny, s_maxx, s_maxy = min(scaled_samples, key=lambda x: x.real).real, min(scaled_samples, key=lambda x: x.imag).imag, max(scaled_samples, key=lambda x: x.real).real, max(scaled_samples, key=lambda x: x.imag).imag
    x_avg = (s_minx + s_maxx)/2
    y_avg = (s_miny + s_maxy)/2
    best_x_avg = WIDTH_RESOLUTION/2
    best_y_avg = HEIGHT_RESOLUTION/2
    centered_samples = [complex(sample.real - x_avg + best_x_avg, sample.imag - y_avg + best_y_avg) for sample in scaled_samples]

    # Now we need to convert the samples to integers
    int_samples = [complex(int(sample.real), int(sample.imag)) for sample in centered_samples]

    # If any two adjacent samples are the same, then we can remove the second one
    deduped_samples = [int_samples[0]]
    for sample in int_samples[1:]:
        if sample != deduped_samples[-1]:
            deduped_samples.append(sample)

    # The samples are now in the control space of the etch-a-sketch
    return deduped_samples

def plot_control_inputs(control_inputs: List[complex], output_file: str):
    """
    Plots the control inputs
    """
    # Plot the control inputs pairwise with the viridis colormap
    for i, (point1, point2) in enumerate(zip(control_inputs, control_inputs[1:])):
        plt.plot([point1.real, point2.real], [point1.imag, point2.imag], color=plt.cm.viridis(i / len(control_inputs)))
    # Plot the bounding box of the control system
    plt.plot([0, WIDTH_RESOLUTION], [0, 0], color="black")
    plt.plot([0, WIDTH_RESOLUTION], [HEIGHT_RESOLUTION, HEIGHT_RESOLUTION], color="black")
    plt.plot([0, 0], [0, HEIGHT_RESOLUTION], color="black")
    plt.plot([WIDTH_RESOLUTION, WIDTH_RESOLUTION], [0, HEIGHT_RESOLUTION], color="black")
    plt.gca().set_aspect('equal', adjustable='box')
    plt.gca().invert_yaxis()
    # Save the plot
    plt.savefig(output_file, dpi=500)
    plt.cla()

def plot_control_animation(control_inputs: List[complex], output_file: str, num_frames=10, total_time=10):
    """
    Plots the control inputs in batches and outputs a gif of the toolpath
    """
    from tqdm import tqdm

    tmp_path = DiskPath("./tmp")
    tmp_path.mkdir(exist_ok=True)
    for file in tmp_path.iterdir():
        file.unlink()
    # Compute the bounding box of the control inputs
    min_x = min(point.real for point in control_inputs)
    max_x = max(point.real for point in control_inputs)
    min_y = min(point.imag for point in control_inputs)
    max_y = max(point.imag for point in control_inputs)
    cutoff_indices = np.linspace(0, len(control_inputs), num_frames, dtype=int, endpoint=True)
    # Now we progressivly add more and more control inputs without clearing the plot to be efficient
    figure, axes = plt.subplots()
    axes.set_xlim(min_x, max_x)
    axes.set_ylim(min_y, max_y)
    figure.gca().set_aspect('equal', adjustable='box')
    figure.gca().invert_yaxis()
    plotted_count = 0
    save_count = 0
    print("Rendering frames...")
    for previous_cutoff, cutoff in tqdm(zip(cutoff_indices, cutoff_indices[1:])):
        for i, (point1, point2) in enumerate(zip(control_inputs[previous_cutoff:cutoff], control_inputs[previous_cutoff+1:cutoff+1])):
            axes.plot([point1.real, point2.real], [point1.imag, point2.imag], color=plt.cm.viridis(plotted_count / len(control_inputs)))
            plotted_count += 1
        # Plot the bounding area for the system
        axes.plot([0, WIDTH_RESOLUTION], [0, 0], color="black")
        axes.plot([0, WIDTH_RESOLUTION], [HEIGHT_RESOLUTION, HEIGHT_RESOLUTION], color="black")
        axes.plot([0, 0], [0, HEIGHT_RESOLUTION], color="black")
        axes.plot([WIDTH_RESOLUTION, WIDTH_RESOLUTION], [0, HEIGHT_RESOLUTION], color="black")
        # Save the plot
        save_name = tmp_path / f"{save_count}.png"
        figure.savefig(save_name, dpi=500)
        save_count += 1
    # Now we make the gif
    files = sorted(tmp_path.iterdir(), key=lambda file: int(file.stem))
    fps = len(files) / total_time
    print(f"Writing gif with {len(files)} frames at {fps} fps")
    with imageio.get_writer(output_file, mode='I', fps=fps) as writer:
        # We want to linger on the last from for 5 seconds
        num_dupe = int(5 * fps)
        files = files + [files[-1]] * num_dupe
        for filename in tqdm(files):
            image = imageio.imread(filename)
            writer.append_data(image)
    # Clean up the tmp directory
    for file in tmp_path.iterdir():
        file.unlink()

def write_control_inputs(control_inputs: List[complex], start_point: complex, output_file: DiskPath):
    """
    Writes the control inputs to a file
    Also writes a hex representation of the control inputs for c++
    """
    hex_file = output_file.with_suffix(".hex")
    with open(output_file, "w") as f:
        for point in [start_point] + control_inputs:
            f.write("{},{}\n".format(point.real, point.imag))
    
    # So that we can copy this into c++, we also write this with escaped characters
    with open(hex_file, "w") as f:
        for point in [start_point] + control_inputs:
            f.write("\\x{:02x}\\x{:02x}\\x{:02x}\\x{:02x}".format(int(point.real) & 0xff, int(point.real) >> 8, int(point.imag) & 0xff, int(point.imag) >> 8))

def complex_list_to_binary(data: List[complex]) -> bytes:
    binary_data = bytearray()
    
    for c in data:
        real = int(c.real)
        imag = int(c.imag)
        
        real_lsb = real & 0xFF
        real_msb = (real >> 8) & 0xFF
        imag_lsb = imag & 0xFF
        imag_msb = (imag >> 8) & 0xFF
        
        binary_data.extend([real_lsb, real_msb, imag_lsb, imag_msb])
    
    return bytes(binary_data)

def get_control_path_from_svg(input_file: DiskPath) -> Tuple[complex, bytes]:
    """
    Gets the binary control path along with the start position from an svg file
    """
    assert input_file.exists() and input_file.is_file() and input_file.suffix == ".svg", f"Invalid input file: {input_file}"

    paths, attrs = svg2paths(input_file)
    paths = get_continuous_paths(paths)
    lines = sample_paths(paths, WIDTH_RESOLUTION, HEIGHT_RESOLUTION, DPR)
    if len(lines) > 1:
        intersections, filtered_intersections, removed_intersections = get_intersections(lines)
        complex_intersection_points = [complex(*point[0]) for point in intersections]
        intersection_parameterization = find_intersection_parameterization(lines, filtered_intersections)
        deduped_intersection_parameterization = mean_shift_cluster(intersection_parameterization)
        intersection_graph = build_intersection_graph(lines, deduped_intersection_parameterization)
        intersection_graph = join_graph_components(intersection_graph, lines)
        raw_control_inputs = compute_tour(intersection_graph, lines)
        control_inputs = process_control_inputs(raw_control_inputs)
    else:
        control_inputs = process_control_inputs(lines[0][1])

    return control_inputs[0], complex_list_to_binary(control_inputs)

def process_svg(input_file: DiskPath, output_file: DiskPath, output_plot: Optional[DiskPath], output_animation: Optional[DiskPath]):
    """
    Processes the SVG file and generates a control file
    """
    assert input_file.exists() and input_file.is_file() and input_file.suffix == ".svg", f"Invalid input file: {input_file}"
    output_file.parent.mkdir(parents=True, exist_ok=True)

    paths, attrs = svg2paths(input_file)
    paths = get_continuous_paths(paths)
    lines = sample_paths(paths, WIDTH_RESOLUTION, HEIGHT_RESOLUTION, DPR)
    if len(lines) > 1:
        intersections, filtered_intersections, removed_intersections = get_intersections(lines)
        complex_intersection_points = [complex(*point[0]) for point in intersections]
        intersection_parameterization = find_intersection_parameterization(lines, filtered_intersections)
        deduped_intersection_parameterization = mean_shift_cluster(intersection_parameterization)
        intersection_graph = build_intersection_graph(lines, deduped_intersection_parameterization)
        intersection_graph = join_graph_components(intersection_graph, lines)
        raw_control_inputs = compute_tour(intersection_graph, lines)
        control_inputs = process_control_inputs(raw_control_inputs)
    else:
        control_inputs = process_control_inputs(lines[0][1])

    write_control_inputs(control_inputs, complex_intersection_points[0], output_file)

    if output_plot is not None:
        plot_control_inputs(control_inputs, output_plot)
    if output_animation is not None:
        plot_control_animation(control_inputs, output_animation)

if __name__ == "__main__":
    import argparse

    parser = argparse.ArgumentParser(description="Converts SVG files to control files for the CNC")
    parser.add_argument("--input-file", type=DiskPath, help="The input SVG file")
    parser.add_argument("--output-file", type=DiskPath, help="The output control file")
    parser.add_argument("--output-plot", type=DiskPath, help="The output plot file")
    parser.add_argument("--output-animation", type=DiskPath, help="The output animation file")
    args = parser.parse_args()
    process_svg(args.input_file, args.output_file, args.output_plot, args.output_animation)
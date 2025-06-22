import numpy as np
from python_tsp.exact import solve_tsp_dynamic_programming
import pandas as pd # Optional: for better data handling if you have many points

# --- 1. Data Representation ---
# Replace this with your actual GPS locations and altitudes.
# You can use a list of dictionaries, a list of lists, or a Pandas DataFrame.
# For simplicity, we'll use a list of dictionaries here.
# 'id' is a logical identifier for your waypoint.
# 'alt' is the altitude in any consistent unit (meters, feet, etc.).
# 'lat' and 'lon' are included for completeness but not used in this specific cost calculation,
# only the 'alt' is used for cost.

gps_data = [
    {'id': 1, 'lat': 8.5678, 'lon': 76.8790, 'alt': 100},
    {'id': 2, 'lat': 8.5700, 'lon': 76.8800, 'alt': 120},
    {'id': 3, 'lat': 8.5650, 'lon': 76.8750, 'alt': 90},
    {'id': 4, 'lat': 8.5720, 'lon': 76.8850, 'alt': 150},
    {'id': 5, 'lat': 8.5600, 'lon': 76.8820, 'alt': 80},
    {'id': 6, 'lat': 8.5750, 'lon': 76.8700, 'alt': 110},
    {'id': 7, 'lat': 8.5620, 'lon': 76.8900, 'alt': 130},
    {'id': 8, 'lat': 8.5690, 'lon': 76.8720, 'alt': 70},
    {'id': 9, 'lat': 8.5710, 'lon': 76.8780, 'alt': 160},
    {'id': 10, 'lat': 8.5640, 'lon': 76.8880, 'alt': 105},
    {'id': 11, 'lat': 8.5730, 'lon': 76.8740, 'alt': 95},
    {'id': 12, 'lat': 8.5680, 'lon': 76.8830, 'alt': 140},
    {'id': 13, 'lat': 8.5610, 'lon': 76.8760, 'alt': 60},
    {'id': 14, 'lat': 8.5740, 'lon': 76.8860, 'alt': 125},
    {'id': 15, 'lat': 8.5660, 'lon': 76.8710, 'alt': 115},
]

# --- 2. Configuration Parameters ---
# Define the factor by which ascent is more costly than descent/level flight.
# For example, if ascent is twice as costly, set ascent_cost_factor = 2.0
ASCENT_COST_FACTOR = 2.0

# Define the starting node ID. Make sure this ID exists in your gps_data.
START_NODE_ID = 1

# --- 3. Cost Matrix Calculation Function ---
def calculate_asymmetric_cost_matrix(locations_data, ascent_factor, start_node_id):
    """
    Calculates the asymmetric cost matrix for the TSP problem.

    Args:
        locations_data (list of dict): A list of dictionaries, each containing
                                      'id' and 'alt' for a waypoint.
        ascent_factor (float): The factor by which ascent is more costly
                               than descent/level flight. (e.g., 2.0)
        start_node_id (int): The ID of the designated starting node.

    Returns:
        numpy.ndarray: A square cost matrix where M[i, j] is the cost
                       of traveling from the waypoint at index i to index j.
    """
    n = len(locations_data)
    cost_matrix = np.full((n, n), np.inf) # Initialize with infinity for non-paths

    # Create a mapping from node ID to its list index for easy lookup
    id_to_index = {loc['id']: i for i, loc in enumerate(locations_data)}
    index_to_id = {i: loc['id'] for i, loc in enumerate(locations_data)}

    # Extract altitudes for direct access by index
    altitudes = [loc['alt'] for loc in locations_data]

    for i in range(n):
        for j in range(n):
            if i == j:
                cost_matrix[i, j] = 0 # Cost to self is 0, or inf if self-loops are strictly forbidden
                                      # For TSP, it's often set to 0 or handled by the algorithm itself.
                                      # Held-Karp usually handles this well with large values or 0.
                                      # Setting to 0 here allows for flexible handling,
                                      # though actual travel to self is disallowed by TSP constraints.
                continue

            alt_i = altitudes[i]
            alt_j = altitudes[j]

            if alt_j > alt_i:  # Ascent
                cost_matrix[i, j] = ascent_factor * (alt_j - alt_i)
            else:  # Descent or Level
                cost_matrix[i, j] = (alt_i - alt_j)

    return cost_matrix, id_to_index, index_to_id

# --- 4. Main Execution ---
if __name__ == "__main__":
    print("--- Calculating Cost Matrix ---")
    cost_matrix, id_to_index, index_to_id = calculate_asymmetric_cost_matrix(gps_data, ASCENT_COST_FACTOR, START_NODE_ID)

    # You can print the cost matrix to review it
    # print("Cost Matrix:\n", cost_matrix)

    # Get the index of the starting node
    start_node_index = id_to_index.get(START_NODE_ID)
    if start_node_index is None:
        raise ValueError(f"Starting Node ID {START_NODE_ID} not found in provided GPS data.")

    print(f"\n--- Solving TSP (Starting from Node {START_NODE_ID}) ---")
    print("Using python-tsp's exact dynamic programming solver (Held-Karp)...")

    # The solve_tsp_dynamic_programming function returns a permutation of indices
    # starting implicitly from the 0th index if not specified.
    # To enforce a specific starting node, we can leverage the optional 'start_node' parameter
    # if available in the library, or more commonly, rotate the result.
    # python-tsp's exact solvers currently don't directly take a start_node parameter for this.
    # The standard way is to find *an* optimal tour and then rotate it to start at the desired node.

    # Find the optimal permutation
    permutation, distance = solve_tsp_dynamic_programming(cost_matrix)

    print(f"\nRaw TSP Result (Indices): Permutation = {permutation}, Total Cost = {distance:.2f}")

    # --- 5. Format and Present the Optimal Path ---

    # Rotate the permutation to start with the desired node.
    # The permutation found by the solver is just a cycle. We need to ensure
    # that our START_NODE_ID is at the beginning of the displayed path.
    if permutation[0] != start_node_index:
        # Find where our start_node_index is in the permutation
        start_idx_in_perm = permutation.index(start_node_index)
        # Rotate the list
        optimal_path_indices = permutation[start_idx_in_perm:] + permutation[:start_idx_in_perm]
    else:
        optimal_path_indices = list(permutation) # Already starts at the desired node

    # Add the starting node at the end to make it cyclic for display
    optimal_path_indices.append(optimal_path_indices[0])

    # Convert indices back to original Node IDs
    optimal_flight_path_ids = [index_to_id[idx] for idx in optimal_path_indices]

    print("\n--- Optimal Flight Path ---")
    print(f"Start Node: {START_NODE_ID}")
    print(f"Optimal Cyclic Path (Node IDs): {' -> '.join(map(str, optimal_flight_path_ids))}")
    print(f"Total Minimum Cost (Height Difference Units): {distance:.2f}")

    print("\n--- Path Details ---")
    current_alt = None
    total_calculated_cost = 0
    path_segments = []

    for i in range(len(optimal_flight_path_ids) - 1):
        from_node_id = optimal_flight_path_ids[i]
        to_node_id = optimal_flight_path_ids[i+1]

        from_idx = id_to_index[from_node_id]
        to_idx = id_to_index[to_node_id]

        segment_cost = cost_matrix[from_idx, to_idx]
        total_calculated_cost += segment_cost

        from_alt = gps_data[from_idx]['alt']
        to_alt = gps_data[to_idx]['alt']

        altitude_change = to_alt - from_alt
        segment_type = "Ascent" if altitude_change > 0 else ("Descent" if altitude_change < 0 else "Level")

        path_segments.append(
            f"  From Node {from_node_id} (Alt: {from_alt}) to Node {to_node_id} (Alt: {to_alt}): "
            f"{segment_type} {'+' if altitude_change > 0 else ''}{altitude_change}, Cost: {segment_cost:.2f}"
        )

    for segment in path_segments:
        print(segment)

    print(f"\nVerification: Sum of segment costs = {total_calculated_cost:.2f}")
    if abs(total_calculated_cost - distance) < 1e-6:
        print("Total calculated cost matches the TSP solver's output. (Good!)")
    else:
        print("Warning: Total calculated cost does not exactly match TSP solver's output. (Check calculations)")

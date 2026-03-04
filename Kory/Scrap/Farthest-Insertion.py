# ============================================================
#  Farthest-Insertion TSP Route
# ============================================================

def plan_farthest_insertion_route(points_latlon, home_latlon):
    pts = np.array(points_latlon)
    home = np.array(home_latlon)

    def haversine(p1, p2):
        R = 6371.0
        lat1, lon1 = np.radians(p1)
        lat2, lon2 = np.radians(p2)
        dlat = lat2 - lat1
        dlon = lon2 - lon1
        a = np.sin(dlat/2)**2 + np.cos(lat1)*np.cos(lat2)*np.sin(dlon/2)**2
        return 2 * R * np.arcsin(np.sqrt(a))

    if len(pts) == 0:
        return np.array([home, home])

    dists = np.array([haversine(home, p) for p in pts])
    sorted_pts = pts[np.argsort(-dists)]

    tour = [home, sorted_pts[0], home]
    used = {tuple(sorted_pts[0])}

    for p in sorted_pts[1:]:
        if tuple(p) in used:
            continue

        best_pos = None
        best_increase = float("inf")

        for i in range(len(tour)-1):
            a = tour[i]
            b = tour[i+1]
            inc = haversine(a, p) + haversine(p, b) - haversine(a, b)

            if inc < best_increase:
                best_increase = inc
                best_pos = i + 1

        tour.insert(best_pos, p)
        used.add(tuple(p))

    return np.array(tour)

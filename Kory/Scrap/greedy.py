# ============================================================
#  Greedy (Nearest-Neighbor) TSP Route
# ============================================================
def plan_greedy_route(points_latlon, home_latlon):
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

    unvisited = pts.tolist()
    tour = [home]
    current = home

    while unvisited:
        dists = [haversine(current, p) for p in unvisited]
        idx = np.argmin(dists)
        next_pt = np.array(unvisited.pop(idx))

        tour.append(next_pt)
        current = next_pt

    tour.append(home)
    return np.array(tour)

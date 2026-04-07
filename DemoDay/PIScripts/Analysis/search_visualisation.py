import numpy as np
import matplotlib.pyplot as plt
from shapely.geometry import Polygon, Point, LineString
from shapely.affinity import rotate, translate

# Camera footprint at 20m altitude
FOOTPRINT_WIDTH = 23.1
FOOTPRINT_HEIGHT = 5.76

def footprint_polygon(x, y, heading_deg):
    """Return a shapely polygon representing the camera footprint."""
    w = FOOTPRINT_WIDTH / 2
    h = FOOTPRINT_HEIGHT / 2

    rect = Polygon([
        (-w, -h),
        ( w, -h),
        ( w,  h),
        (-w,  h)
    ])

    rect = rotate(rect, heading_deg, origin=(0,0), use_radians=False)
    rect = translate(rect, xoff=x, yoff=y)
    return rect

def rasterise_coverage(waypoints_xy, headings_deg, resolution=1.0):
    """
    waypoints_xy: list of (x,y) in meters
    headings_deg: list of headings for each segment
    resolution: grid cell size in meters
    """
    xs = [p[0] for p in waypoints_xy]
    ys = [p[1] for p in waypoints_xy]

    xmin, xmax = min(xs)-30, max(xs)+30
    ymin, ymax = min(ys)-30, max(ys)+30

    nx = int((xmax - xmin) / resolution)
    ny = int((ymax - ymin) / resolution)

    grid = np.zeros((ny, nx))

    for i in range(len(waypoints_xy)-1):
        x1, y1 = waypoints_xy[i]
        x2, y2 = waypoints_xy[i+1]
        heading = headings_deg[i]

        # sample along segment
        seg = LineString([(x1,y1),(x2,y2)])
        length = seg.length
        steps = max(2, int(length / (resolution/2)))

        for t in np.linspace(0,1,steps):
            x, y = seg.interpolate(t, normalized=True).coords[0]
            fp = footprint_polygon(x, y, heading)

            # rasterise
            for ix in range(nx):
                for iy in range(ny):
                    cx = xmin + ix*resolution
                    cy = ymin + iy*resolution
                    cell = Point(cx, cy)
                    if fp.contains(cell):
                        grid[iy, ix] += 1

    return grid, (xmin, xmax, ymin, ymax)

def plot_heatmap(grid, bounds, title="Coverage Heatmap"):
    xmin, xmax, ymin, ymax = bounds
    plt.figure(figsize=(10,8))
    plt.imshow(grid, extent=[xmin,xmax,ymin,ymax], origin='lower', cmap='hot')
    plt.colorbar(label="Coverage Count")
    plt.title(title)
    plt.xlabel("X (m)")
    plt.ylabel("Y (m)")
    plt.show()

def plot_gaps_and_overlaps(grid, bounds):
    xmin, xmax, ymin, ymax = bounds
    gaps = (grid == 0)
    overlaps = (grid > 1)

    plt.figure(figsize=(10,8))
    plt.imshow(gaps, extent=[xmin,xmax,ymin,ymax], origin='lower', cmap='Blues')
    plt.title("Gaps (blue = uncovered)")
    plt.show()

    plt.figure(figsize=(10,8))
    plt.imshow(overlaps, extent=[xmin,xmax,ymin,ymax], origin='lower', cmap='Greens')
    plt.title("Overlaps (green = multiple coverage)")
    plt.show()

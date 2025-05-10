import rasterio
from rasterio.plot import show
from rasterio.transform import rowcol
from pyproj import Transformer
import matplotlib.pyplot as plt

# Example lat/lon points
lat_lon_points = [
    (40.2231137, -111.6273118),  # (lat, lon)
    (40.2233902, -111.6271029)
]

# Open GeoTIFF
with rasterio.open("slate_canyon.tif") as src:
    data = src.read(1)
    transform = src.transform

    # Print CRS to understand what projection we're working with
    print("TIFF CRS:", src.crs)

    # Transformer from WGS84 (lat/lon) to the TIFF's CRS
    transformer = Transformer.from_crs("EPSG:4326", src.crs, always_xy=True)

    # Convert to the TIFF's CRS
    transformed_points = [transformer.transform(lon, lat) for lat, lon in lat_lon_points]

    # Convert to pixel coordinates
    pixel_coords = [rowcol(transform, x, y) for x, y in transformed_points]

    # Crop the image around those points (optional)
    min_row = max(0, min(row for row, col in pixel_coords) - 10)
    max_row = min(data.shape[0], max(row for row, col in pixel_coords) + 10)
    min_col = max(0, min(col for row, col in pixel_coords) - 10)
    max_col = min(data.shape[1], max(col for row, col in pixel_coords) + 10)
    cropped = data[min_row:max_row, min_col:max_col]

    # Plot cropped data
    plt.imshow(cropped, cmap='terrain', origin='upper')
    # plt.imshow(cropped, cmap='terrain', extent=(min_col, max_col, max_row, min_row))
    plt.colorbar(label='Elevation')
    plt.title('Cropped Topography with Lat/Lon Points')

    # # Offset point locations to match cropped image
    # for row, col in pixel_coords:
    #     plt.plot(col - min_col, row - min_row, marker='o', color='red', markersize=5)
    for row, col in pixel_coords:
        plt.plot(col - min_col, row - min_row, marker='o', color='red')
    plt.show()

    for i, (row, col) in enumerate(pixel_coords):
        print(f"Original pixel: row={row}, col={col}")
        print(f"Cropped-relative pixel: row={row - min_row}, col={col - min_col}")


print("Transform matrix:", transform)
print("Pixel coordinates:", pixel_coords)
print("Cropped bounds (rows, cols):", (min_row, max_row), (min_col, max_col))

from turfpy.measurement import boolean_point_in_polygon
from geojson import Point, Polygon, Feature

point = Feature(geometry=Point((10.04228, 76.35431)))
polygon = Polygon(
    [
        [
            (10.0747726, 76.3380549),
            ( 10.0637448, 76.3645917),
            ( 10.0554795, 76.3557201),
            ( 10.0551706, 76.3218176),
            (10.0747726, 76.3380549)
        ]
    ]
)
print(boolean_point_in_polygon(point, polygon))


#pragma once

#include <godot_cpp/classes/object.hpp>
#include <godot_cpp/variant/dictionary.hpp>
#include <godot_cpp/variant/utility_functions.hpp>
#include <godot_cpp/core/class_db.hpp>
#include <vector>

using namespace godot;

class Poisson : public Object {
    GDCLASS(Poisson, Object);
    
    PackedVector2Array _shape;
    PackedVector2Array _stored_bounds;
    int _type; //0 = Circular, 1 = Rectangular, 2 = Polygonal
    float _poission_radius;
    int _retries;
    float _circle_radius;

protected:
    static void _bind_methods();

    PackedVector2Array _find_polygon_bounds(PackedVector2Array &shape);

    bool _is_in_shape(const PackedVector2Array &shape, Vector2 &point);

    Vector2 _rand_sphere_annulus(float radius);

    Vector2 _random_point_in_shape(const PackedVector2Array &shape, int retries);

    int _world_to_grid_index(Vector2 world, Vector2 origin, float cell_size, int cols, int rows);

    bool _is_valid(Vector2 candidate, const std::vector<int> &grid, const PackedVector2Array &points, float min_dist, Vector2 origin, float cell_size, int cols, int rows);

public:
    void set_shape(const PackedVector2Array &shape);
    void set_rect(int length, int width);
    void set_circle(float radius);
    PackedVector2Array get_shape() const;
    PackedVector2Array get_bounds();

    PackedVector2Array generate(float radius, int retries, const PackedVector2Array &append_to = PackedVector2Array()); 

    Poisson();
};

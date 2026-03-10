#include "poisson.h"

void Poisson::_bind_methods() {
    //D_METHOD("gdscript_name", "arg1_name", "arg2_name")
    ClassDB::bind_method(D_METHOD("set_shape", "shape"), &Poisson::set_shape);
    ClassDB::bind_method(D_METHOD("set_rect", "length", "width"), &Poisson::set_rect);
    ClassDB::bind_method(D_METHOD("set_circle", "radius"), &Poisson::set_circle);

    ClassDB::bind_method(D_METHOD("get_shape"), &Poisson::get_shape);
    ClassDB::bind_method(D_METHOD("get_bounds"), &Poisson::get_bounds);

    //To give 'append_to' a default value in GDScript, we use DEFVAL() at the end.
    ClassDB::bind_method(
        D_METHOD("generate", "min_dist", "retries", "append_to"), 
        &Poisson::generate, 
        DEFVAL(PackedVector2Array())
    );
}

void Poisson::set_shape(const PackedVector2Array &shape) {
    //Sets the internal shape to a polygon
    _shape = shape;
    _type = 2;
};

void Poisson::set_rect(int length, int width) {
    //Sets the internal shape to a rectangle of length and width
    _shape.clear();
    _shape.resize(4);
    _shape.set(0, Vector2(0, 0));
    _shape.set(1, Vector2(length, 0));
    _shape.set(2, Vector2(length, width));
    _shape.set(3, Vector2(0, width));
    _type = 1;
};

void Poisson::set_circle(float radius) {
    //Setsthe internal shape to a circle (Rect with special handling)
    _circle_radius = radius;
    set_rect(radius * 2, radius * 2);
    _type = 0;
};

PackedVector2Array Poisson::get_shape() const {
    //Getter for the stored shape
    return _shape;
};

PackedVector2Array Poisson::generate(float min_dist, int retries, const PackedVector2Array &append_to) {
    PackedVector2Array bb = _find_polygon_bounds(_shape);

    float cell_size = min_dist / Math::sqrt(2.0f);

    int bb_width = bb[2].x - bb[0].x;
    int bb_height = bb[2].y - bb[0].y;

    int cols = Math::ceil(bb_width / cell_size);
    int rows = Math::ceil(bb_height / cell_size);

    int arr_len = cols * rows;
    //index = y * cols + x | y is row index, x is column index

    std::vector<int> index_grid(arr_len, -1);
    PackedVector2Array result;
    std::vector<int> active_heap;
    int index = 0;

    if(!append_to.is_empty()) {
        for(int i = 0; i < append_to.size(); i++) {
            index_grid[_world_to_grid_index(append_to[i], bb[0], cell_size, cols, rows)] = index;
            index++;
        }
        result = append_to;
    }
    
    Vector2 start = _random_point_in_shape(get_shape(), retries);
    if(start.x == -99999) {
        ERR_PRINT("Poisson Error: Could not find a valid starting point within the shape after max retries. Shape might be too small or complex.");
        return PackedVector2Array();
    }
    while(!_is_valid(start, index_grid, result, min_dist, bb[0], cell_size, cols, rows)) start = _random_point_in_shape(get_shape(), retries);
    result.push_back(start);
    active_heap.push_back(index);
    index++;
    index_grid[_world_to_grid_index(start, bb[0], cell_size, cols, rows)] = index;
    while(!active_heap.empty()) {
        int idx = active_heap[godot::UtilityFunctions::randi_range(0,active_heap.size()-1)];
        Vector2 active = result[idx];
        int k = 0;
        while(k < retries) {
            Vector2 candidate = active + _rand_sphere_annulus(min_dist);
            if(_is_in_shape(_shape, candidate) && _is_valid(candidate, index_grid, result, min_dist, bb[0], cell_size, cols, rows)) {
                result.push_back(candidate);
                active_heap.push_back(index);
                index++;
                index_grid[_world_to_grid_index(candidate, bb[0], cell_size, cols, rows)] = index;
                k = 0;
            } else {
                k++;
            }
        }
        active_heap[idx] = active_heap.back();
        active_heap.pop_back();
    }
    return result;
};

PackedVector2Array Poisson::_find_polygon_bounds(PackedVector2Array &shape) { //The AABB of the shape
    if(_type == 0 || _type == 1) return get_shape(); //Just use the shape for Circles and Squares

    float maxX = shape[0].x, maxY = shape[0].y;
    float minX = shape[0].x, minY = shape[0].y;

    for (int i = 1; i < shape.size(); i++) {
        Vector2 point = shape[i];
        if (point.x > maxX) maxX = point.x;
        else if (point.x < minX) minX = point.x;
        if (point.y > maxY) maxY = point.y;
        else if (point.y < minY) minY = point.y;
    }

    PackedVector2Array vec;
    vec.resize(4);
    vec.set(0, Vector2(minX, minY));
    vec.set(1, Vector2(minX, maxY));
    vec.set(2, Vector2(maxX, maxY));
    vec.set(3, Vector2(maxX, minY));

    _stored_bounds = vec;

    return(vec);
};

bool Poisson::_is_in_shape(const PackedVector2Array &shape, Vector2 &point) {
    PackedVector2Array bounds = get_bounds();
    
    if(//Fast AABB test first
        point.x < bounds[0].x ||
        point.y < bounds[0].y ||
        point.x > bounds[2].x ||
        point.y > bounds[2].y
    ) return false;

    if(_type == 1) return true; //Return true if AABB test passed

    if(_type == 0){ //Circle Test
        Vector2 centre = bounds[0].lerp(bounds[2], 0.5f);
        return (point.distance_squared_to(centre) <= (_circle_radius * _circle_radius));
    }
    bool inside = false;
    for (int i = 0, j = shape.size() - 1; i < shape.size(); j = i++) { //j starts as the end point, and then updates to i before it is incremented each loop
        Vector2 pi = shape[i];
        Vector2 pj = shape[j];
        
        //Raycast check, first see if we are above or below both points in the line, and as such, not intersecting
        if (((pi.y > point.y) != (pj.y > point.y)) && (point.x < (pj.x - pi.x) * (point.y - pi.y) / (pj.y - pi.y) + pi.x)) {
            inside = !inside;
        }
    }
    return inside;
};

Vector2 Poisson::_rand_sphere_annulus(float radius) {
    float range = godot::UtilityFunctions::randf_range(radius, 2.0f * radius);
    float angle = godot::UtilityFunctions::randf_range(0.0f , Math_TAU);
    return Vector2(range * Math::cos(angle), range * Math::sin(angle));
};

Vector2 Poisson::_random_point_in_shape(const PackedVector2Array &shape, int retries) {
    PackedVector2Array bounds = get_bounds();
    Vector2 point;
    point.x = godot::UtilityFunctions::randf_range(bounds[0].x, bounds[2].x);
    point.y = godot::UtilityFunctions::randf_range(bounds[0].y, bounds[2].y);
    int k = 0;
    while(!_is_in_shape(shape, point) && k < retries)
    {
        point.x = godot::UtilityFunctions::randf_range(bounds[0].x, bounds[2].x);
        point.y = godot::UtilityFunctions::randf_range(bounds[0].y, bounds[2].y);
        k++;
    }
    if(k >= retries) return Vector2(-99999, -99999);
    return point;
}

int Poisson::_world_to_grid_index(Vector2 world, Vector2 origin, float cell_size, int cols, int rows) {
    int x = (int)((world.x - origin.x) / cell_size);
    int y = (int)((world.y - origin.y) / cell_size);

    return Math::clamp(y, 0, rows-1) * cols + Math::clamp(x, 0, cols-1);
}

PackedVector2Array Poisson::get_bounds() {
    return _stored_bounds;
}

bool Poisson::_is_valid(Vector2 candidate, const std::vector<int> &grid, const PackedVector2Array &points, float min_dist, Vector2 origin, float cell_size, int cols, int rows) {
    if(points.size() < 1) return true;

    //Get the grid coordinates
    int gx = (int)((candidate.x - origin.x) / cell_size);
    int gy = (int)((candidate.y - origin.y) / cell_size);
    
    float min_dist_sq = min_dist * min_dist;

    //Search the 5x5 neighborhood
    for (int y = gy - 2; y <= gy + 2; y++) {
        for (int x = gx - 2; x <= gx + 2; x++) {
            
            //Boundary check: Skip if this neighbor cell is outside the grid
            if (x < 0 || x >= cols || y < 0 || y >= rows) continue;

            int cell_index = y * cols + x;
            int point_index = grid[cell_index];

            //If the cell is not empty (-1), check the distance
            if (point_index != -1) {
                if (candidate.distance_squared_to(points[point_index]) < min_dist_sq) {
                    return false; //There is a point that is too close
                }
            }
        }
    }
    return true; //All neighbors are at a safe distance
}

Poisson::Poisson() {
    _type = 1; // Default to rectangular
    _retries = 30;
    _circle_radius = 0.0f;
    _poission_radius = 0.0f;
    
    set_rect(10, 10);
};
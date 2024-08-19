@tool
extends Node3D

@export var set_start: bool = false : set = start
@export var border_size: int = 20 : set = set_border_size
@onready var grid_map: GridMap = $GridMap

@export var room_margin: int = 1
@export var max_recursion: int = 10
@export var set_number_rooms: int = 4
@export var set_min_size: int = 2
@export var set_max_size: int = 4
@export_range(0, 1) var survival_chance: float = 0.25

@export_multiline var cust_seed: String = "" : set = set_seed




var room_tiles : Array[PackedVector3Array]
var room_position : PackedVector3Array

func start(_val: bool) -> void:
	if Engine.is_editor_hint():
		generate()
	

func set_seed(val: String) -> void:
	cust_seed = val
	seed(cust_seed.hash())
	

func set_border_size(size: int) -> void:
	border_size = size
	visualize_border()
	
func visualize_border() -> void:
	grid_map.clear()
	for i in range(-1,border_size+1):
		grid_map.set_cell_item(Vector3i(i,0,-1), 3)
		grid_map.set_cell_item(Vector3i(-1,0,i), 3)
		grid_map.set_cell_item(Vector3i(border_size,0,i), 3)
		grid_map.set_cell_item(Vector3i(i,0,border_size), 3)

func generate() -> void:
	room_position.clear()
	room_tiles.clear()
	if cust_seed:
		set_seed(cust_seed)
	
	visualize_border()
	for i in set_number_rooms:
		make_room(max_recursion)
	
	var room_packedvector2 : PackedVector2Array = []
	var mst_graph := AStar2D.new()
	var delaunay_graph := AStar2D.new()
	
	#Transporting Vector3 points to Vector2
	for points in room_position:
		room_packedvector2.append(Vector2(points.x, points.z))
		delaunay_graph.add_point(delaunay_graph.get_available_point_id(), Vector2(points.x, points.z))
		mst_graph.add_point(mst_graph.get_available_point_id(), Vector2(points.x, points.z))
	
	#Creation of Delaunay Triangulation
	var delaunay_triang : Array = Array(Geometry2D.triangulate_delaunay(room_packedvector2))
	
	for t in delaunay_triang.size()/3:
		var point_1 : int = delaunay_triang.pop_front()
		var point_2 : int = delaunay_triang.pop_front()
		var point_3 : int = delaunay_triang.pop_front()
		
		delaunay_graph.connect_points(point_1, point_2)
		delaunay_graph.connect_points(point_1, point_3)
		delaunay_graph.connect_points(point_2, point_3)
		
	
	var visited_points : Array = []
	visited_points.append(randi() % room_position.size())
	while visited_points.size() != mst_graph.get_point_count():
		var possible_connections : Array[PackedInt32Array] = []
		for vpoint in visited_points:
			for c in delaunay_graph.get_point_connections(vpoint):
				if !visited_points.has(c):
					var p_con : PackedInt32Array = [vpoint, c]
					possible_connections.append(p_con)
		var connection : PackedInt32Array = possible_connections.pick_random()
		for pc in possible_connections:
			if room_packedvector2[pc[0]].distance_squared_to(room_packedvector2[pc[1]]) <\
			room_packedvector2[connection[0]].distance_squared_to(room_packedvector2[connection[1]]):
				connection = pc
		
		visited_points.append(connection[1])
		mst_graph.connect_points(connection[0], connection[1])
		delaunay_graph.disconnect_points(connection[0], connection[1])
	
	var hallway_graph : AStar2D = mst_graph
	
	for p in delaunay_graph.get_point_ids():
		for c in delaunay_graph.get_point_connections(p):
			if c>p:
				var kill : float = randf()
				if survival_chance > kill:
					hallway_graph.connect_points(p, c)
	
	create_hallways(hallway_graph)
	

func create_hallways(graph: AStar2D) -> void:
	var hallways : Array[PackedVector3Array] = []
	for point in graph.get_point_ids():
		for connection in graph.get_point_connections(point):
			if connection>point:
				var room_from: PackedVector3Array = room_tiles[point]
				var room_to: PackedVector3Array = room_tiles[connection]
				var tile_from: Vector3 = room_from[0]
				var tile_to: Vector3 = room_to[0]
				for tile in room_from:
					if tile.distance_squared_to(room_position[connection]) < tile_from.distance_squared_to(room_position[connection]):
						tile_from = tile
				for tile in room_to:
					if tile.distance_squared_to(room_position[point]) < tile_to.distance_squared_to(room_position[point]):
						tile_to = tile
				var hall: PackedVector3Array = [tile_from, tile_to]
				hallways.append(hall)
				grid_map.set_cell_item(tile_from, 2)
				grid_map.set_cell_item(tile_to, 2)
	
	var hallway_astar: AStarGrid2D = AStarGrid2D.new()
	hallway_astar.size = Vector2i.ONE * border_size
	hallway_astar.update()
	hallway_astar.diagonal_mode = AStarGrid2D.DIAGONAL_MODE_NEVER
	hallway_astar.default_compute_heuristic = AStarGrid2D.HEURISTIC_MANHATTAN
	
	for t in grid_map.get_used_cells_by_item(0):
		hallway_astar.set_point_solid(Vector2i(t.x, t.z))
	
	for h in hallways:
		var pos_from: Vector2i = Vector2i(h[0].x, h[0].z)
		var pos_to: Vector2i = Vector2i(h[1].x, h[1].z)
		var hall: PackedVector2Array = hallway_astar.get_point_path(pos_from, pos_to)
		
		for t in hall:
			var position: Vector3i = Vector3i(t.x, 0, t.y)
			if grid_map.get_cell_item(position) < 0:
				grid_map.set_cell_item(position, 1)
				
	
	

func make_room(rec: int) -> void:
	if rec <= 0:
		return
	
	var width: int = (randi() % (set_max_size - set_min_size)) + set_min_size
	var height: int = (randi() % (set_max_size - set_min_size)) + set_min_size
	
	var start_position: Vector3i
	start_position.x = randi() % (border_size - width + 1)
	start_position.z = randi() % (border_size - height + 1)
	
	for row in range(-room_margin, height+room_margin):
		for column in range(-room_margin, width+room_margin):
			var position: Vector3i = start_position + Vector3i(column, 0, row)
			if grid_map.get_cell_item(position) == 0:
				make_room(rec - 1)
				return

	var room : PackedVector3Array = []
	for row in height:
		for column in width:
			var position: Vector3i = start_position + Vector3i(column, 0, row)
			grid_map.set_cell_item(position, 0)
			room.append(position)
	room_tiles.append(room)
	var middle_point_x : float = start_position.x + (float(width)/2)
	var middle_point_z : float = start_position.z + (float(height)/2)
	var middle_point : Vector3 = Vector3(middle_point_x, 0, middle_point_z)
	room_position.append(middle_point)

#include "../include/Planning.hpp"

PlanningNode::PlanningNode() :
    rclcpp::Node("planning_node") {

        // Client for map
        map_client_ = this->create_client<nav_msgs::srv::GetMap>("/map_server/map");

        // Service for path
        plan_service_ = this->create_service<nav_msgs::srv::GetPlan>(
            "plan_path",
            std::bind(&PlanningNode::planPath, this, std::placeholders::_1, std::placeholders::_2)
        );
        
        RCLCPP_INFO(get_logger(), "Služba plan_path pripravená.");
        
        
        // Publisher for path
        path_pub_ = this->create_publisher<nav_msgs::msg::Path>("/path", 10);



        RCLCPP_INFO(get_logger(), "Planning node started.");

        // Connect to map server
        while (!map_client_->wait_for_service(std::chrono::seconds(1))) {
            RCLCPP_INFO(get_logger(), "Čakám na službu 'map'...");
        }

        // Request map
        auto request = std::make_shared<nav_msgs::srv::GetMap::Request>();

        auto future = map_client_->async_send_request(request,
            std::bind(&PlanningNode::mapCallback, this, std::placeholders::_1));
        
        RCLCPP_INFO(get_logger(), "Odoslal som požiadavku na mapu.");
    }

void PlanningNode::mapCallback(rclcpp::Client<nav_msgs::srv::GetMap>::SharedFuture future) {

    auto response = future.get();
    if (response) {
        map_ = response->map;
        RCLCPP_INFO(get_logger(), "Mapa úspešne načítaná: %d x %d",
            map_.info.width, map_.info.height);
    } else {
        RCLCPP_ERROR(get_logger(), "Nepodarilo sa načítať mapu.");
    }

    // ********
    // * Help *
    // ********
    /*
    auto response = future.get();
    if (response) {
        ...
    }
    */
}

void PlanningNode::planPath(const std::shared_ptr<nav_msgs::srv::GetPlan::Request> request, std::shared_ptr<nav_msgs::srv::GetPlan::Response> response) {

    RCLCPP_INFO(get_logger(), "Volanie služby plan_path prijaté.");

    // Naplníme dummy cestu medzi start a goal
    geometry_msgs::msg::PoseStamped start = request->start;
    geometry_msgs::msg::PoseStamped goal = request->goal;

    nav_msgs::msg::Path path;
    path.header.frame_id = "map";
    path.header.stamp = this->get_clock()->now();

    // A* algoritmus a smoothing
    aStar(request->start, request->goal);
    smoothPath();  // Ak je prázdna, nevadí

    // Skontroluj, či je cesta naplnená
    if (path_.poses.empty()) {
        RCLCPP_WARN(get_logger(), "Cesta je prázdna.");
    } else {
        RCLCPP_INFO(get_logger(), "Cesta obsahuje %ld bodov.", path_.poses.size());
    }

    // Uloženie naplánovanej cesty do odpovede
    response->plan = path_;
    
    // Publikovanie cesty
    path_pub_->publish(path_);

    // Uložíme aj do členskej pre publisher (do budúcna)
    path_ = path;
}

void PlanningNode::dilateMap() {
    // add code here

    // ********
    // * Help *
    // ********
    /*
    nav_msgs::msg::OccupancyGrid dilatedMap = map_;
    ... processing ...
    map_ = dilatedMap;
    */
}

void PlanningNode::aStar(const geometry_msgs::msg::PoseStamped &start, const geometry_msgs::msg::PoseStamped &goal) {
    int width = map_.info.width;
    int height = map_.info.height;
    float resolution = map_.info.resolution;
    float origin_x = map_.info.origin.position.x;
    float origin_y = map_.info.origin.position.y;

    auto worldToMap = [&](float wx, float wy, int &mx, int &my) {
        mx = static_cast<int>((wx - origin_x) / resolution);
        my = static_cast<int>((wy - origin_y) / resolution);
    };

    int sx, sy, gx, gy;
    worldToMap(start.pose.position.x, start.pose.position.y, sx, sy);
    worldToMap(goal.pose.position.x, goal.pose.position.y, gx, gy);

    const int8_t OBSTACLE_THRESHOLD = 50;

    auto isFree = [&](int x, int y) {
        // Skontrolujeme, či je pozícia v platnom rozsahu
        if (x < 0 || x >= width || y < 0 || y >= height) return false;
        int index = y * width + x;
        int8_t val = map_.data[index];
        return val >= 0 && val < OBSTACLE_THRESHOLD;
    };

    auto heuristic = [&](int x1, int y1, int x2, int y2) {
        return std::hypot(x1 - x2, y1 - y2);
    };

    std::vector<std::shared_ptr<Cell>> openList;
    std::vector<bool> closedList(width * height, false);

    auto startCell = std::make_shared<Cell>(sx, sy);
    startCell->g = 0;
    startCell->h = heuristic(sx, sy, gx, gy);
    startCell->f = startCell->g + startCell->h;
    openList.push_back(startCell);

    std::shared_ptr<Cell> goalCell = nullptr;

    std::array<std::pair<int, int>, 8> directions = {{
        {0, 1}, {1, 0}, {0, -1}, {-1, 0},
        {1, 1}, {-1, 1}, {1, -1}, {-1, -1}
    }};

    while (!openList.empty() && rclcpp::ok()) {
        auto currentIt = std::min_element(openList.begin(), openList.end(), [](auto a, auto b) {
            return a->f < b->f;
        });

        std::shared_ptr<Cell> current = *currentIt;
        openList.erase(currentIt);
        closedList[current->y * width + current->x] = true;

        if (current->x == gx && current->y == gy) {
            goalCell = current;
            break;
        }

        for (auto [dx, dy] : directions) {
            int nx = current->x + dx;
            int ny = current->y + dy;

            if (!isFree(nx, ny) || closedList[ny * width + nx]) continue;

            bool inOpen = false;
            for (auto &cell : openList) {
                if (cell->x == nx && cell->y == ny) {
                    inOpen = true;
                    break;
                }
            }

            if (!inOpen) {
                auto neighbor = std::make_shared<Cell>(nx, ny);
                neighbor->g = current->g + heuristic(current->x, current->y, nx, ny);
                neighbor->h = heuristic(nx, ny, gx, gy);
                neighbor->f = neighbor->g + neighbor->h;
                neighbor->parent = current;
                openList.push_back(neighbor);
            }
        }
    }

    path_.poses.clear();

    if (goalCell) {
        std::vector<geometry_msgs::msg::PoseStamped> waypoints;
        auto cell = goalCell;

        while (cell) {
            geometry_msgs::msg::PoseStamped pose;
            pose.header.frame_id = "map";
            pose.pose.position.x = cell->x * resolution + origin_x + resolution / 2.0;
            pose.pose.position.y = cell->y * resolution + origin_y + resolution / 2.0;
            pose.pose.orientation.w = 1.0;
            waypoints.push_back(pose);
            cell = cell->parent;
        }

        std::reverse(waypoints.begin(), waypoints.end());
        path_.header.frame_id = "map";
        path_.header.stamp = this->get_clock()->now();
        path_.poses = waypoints;

        RCLCPP_INFO(get_logger(), "Cesta naplánovaná cez %ld bodov.", path_.poses.size());
        // Publikujeme path_ po jej naplnení
        path_pub_->publish(path_);
    } else {
        RCLCPP_WARN(get_logger(), "Cesta sa nepodarila nájsť.");
    }

    // ********
    // * Help *
    // ********
    /*
    Cell cStart(...x-map..., ...y-map...);
    Cell cGoal(...x-map..., ...y-map...);

    std::vector<std::shared_ptr<Cell>> openList;
    std::vector<bool> closedList(map_.info.height * map_.info.width, false);

    openList.push_back(std::make_shared<Cell>(cStart));

    while(!openList.empty() && rclcpp::ok()) {
        ...
    }

    RCLCPP_ERROR(get_logger(), "Unable to plan path.");
    */
}

void PlanningNode::smoothPath() {
    // add code here

    // ********
    // * Help *
    // ********
    /*
    std::vector<geometry_msgs::msg::PoseStamped> newPath = path_.poses;
    ... processing ...
    path_.poses = newPath;
    */
}
/*
Cell::Cell(int c, int r) {
    // add code here
}*/

Cell::Cell(int c, int r) : x(c), y(r), g(0), h(0), f(0), parent(nullptr) {
    // Initialize all member variables
}

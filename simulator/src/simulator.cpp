#include "simulator.hpp"



Simulator::Simulator(int& width,int& height, string& file_name ):window(sf::VideoMode(width,height),"waypoint visualized"){
    load_waypoints(file_name);
    window.setFramerateLimit(60);
    view.setSize(window.getSize().x,window.getSize().y);
    view.setCenter(window.getSize().x/2.f + 100,window.getSize().y/2.f-80);
    window.setView(view);
    //creates an off-screen texture (like a hidden canvas) .
    waypointTexture.create(width,height);
    waypointTexture.clear(sf::Color::Black);
    scale = 10.0f;
    offset_x = window.getSize().x / 2;
    offset_y = window.getSize().y / 2;
    for(const auto& wp: waypoints){
        sf::CircleShape waypoint(5); //5 pixel ,  5 is radius of circle
        waypoint.setFillColor(sf::Color::White);
        //waypoint.setPosition(wp.x-5, wp.y-5); // The circle's top-left corner will be placed at (wp.x, wp.y) offset need to make center at wp.x and wp.y
        waypoint.setPosition(wp.x * scale + offset_x, wp.y * scale + offset_y);
        
        waypointTexture.draw(waypoint);
    }

    waypointTexture.display();
    //sprite in SFML is just a drawable object that holds a texture
    
    //sprite is lighter and faster to draw than individual shapes
    //setTexture() binds this texture to the sprite, so the sprite will now display the texture
    waypointSprite.setTexture(waypointTexture.getTexture());
    
}

void Simulator::load_waypoints(const string& file_name){
    std::ifstream file(file_name);
    
    if(!file){
        std::cerr<<"Failed to open file " << file_name<<std::endl;
        return;
    }
    std::string line;
    int i=0;
    while(std::getline(file,line)){
        
        std::stringstream ss(line);
        string token;
        vector<float> v;
        while (std::getline(ss, token, ',')) {
               v.push_back(stof(token));
        }
        waypoints.emplace_back(v[0], v[1]);
        // std::cout << "Stream content: " << ss.str() << std::endl;
        // if( ss>>x>>y>>z){
        //     cout<<++i;
        //     waypoints.emplace_back(x,y);

        // }
    }
    file.close();
    std::cout<<"Loaded "<< waypoints.size()<< "waypoints"<< std::endl;
}


void Simulator::run(const double& car_x,const double& car_y)
    
    {

        sf::Event event;
        while(window.pollEvent(event)){
            if(event.type==sf::Event::Closed){
                window.close();
            }
        if(event.type == sf::Event::MouseWheelScrolled)
            {
                if(event.mouseWheelScroll.delta>0){
                    zoom_factor*=0.9f;
                }else{zoom_factor*=1.1f;}
                view.setSize(window.getSize().x*zoom_factor,window.getSize().y*zoom_factor);
                window.setView(view);
            }
        }

        window.clear();
        window.draw(waypointSprite);
        sf::CircleShape car(4);
        car.setFillColor(sf::Color::Blue);
        car.setPosition(car_x* scale + offset_x, car_y* scale + offset_y);
        window.draw(car);

        // Step 3: Display frame
        window.display();    
    
    }




// Simulator::Simulator(int& width,int& height, string& file_name )
//     : window(sf::VideoMode(width, height), "Waypoint Visualizer") {
//     load_waypoints(file_name);
//     window.setFramerateLimit(60);

//     // Create an offscreen texture
//     if (!waypointTexture.create(width, height)) {
//         std::cerr << "Failed to create waypointTexture" << std::endl;
//         return;
//     }

//     float scale = 10.0f;
//     float offset_x = window.getSize().x / 2;
//     float offset_y = window.getSize().y / 2;

//     waypointTexture.clear(sf::Color::Black);
//     for (const auto& wp : waypoints) {
//         std::cout << "Drawing waypoint at: (" << wp.x << ", " << wp.y << ")" << std::endl;
//         sf::CircleShape waypoint(2);
//         waypoint.setFillColor(sf::Color::White);
//         waypoint.setPosition(wp.x * scale + offset_x, wp.y * scale + offset_y);
//         waypointTexture.draw(waypoint);
//     }

//     waypointTexture.display();
//     waypointSprite.setTexture(waypointTexture.getTexture());
// }

// void Simulator::load_waypoints(const std::string& file_name) {
//     std::ifstream file(file_name);
//     if (!file) {
//         std::cerr << "Failed to open file: " << file_name << std::endl;
//         return;
//     }

//     std::string line;
//     while (std::getline(file, line)) {
//         std::stringstream ss(line);
//         std::string token;
//         std::vector<float> v;

//         while (std::getline(ss, token, ',')) {
//             try {
//                 v.push_back(std::stof(token));
//             } catch (const std::exception& e) {
//                 std::cerr << "Conversion error: " << e.what() << std::endl;
//             }
//         }

//         if (v.size() >= 2) {
//             waypoints.emplace_back(v[0], v[1]);
//             std::cout << "Loaded waypoint: (" << v[0] << ", " << v[1] << ")" << std::endl;
//         }
//     }

//     file.close();
//     std::cout << "Loaded " << waypoints.size() << " waypoints." << std::endl;
// }

// void Simulator::run(const double& car_x, const double& car_y) {
//     sf::Event event;
//     while (window.pollEvent(event)) {
//         if (event.type == sf::Event::Closed) {
//             window.close();
//         }
//     }

//     window.clear();

//     // ✅ Draw waypoint sprite
//     window.draw(waypointSprite);

//     // ✅ Draw car
//     sf::CircleShape car(10);
//     car.setFillColor(sf::Color::Blue);
//     car.setPosition(car_x - 10, car_y - 10);
//     window.draw(car);

//     window.display();
// }



// if (waypoints.size() > 1) {
//     for (size_t i = 0; i < waypoints.size() - 1; ++i) {
//         sf::Vector2f start = {
//             waypoints[i].x * scale + offset_x,
//             waypoints[i].y * scale + offset_y
//         };
//         sf::Vector2f end = {
//             waypoints[i + 1].x * scale + offset_x,
//             waypoints[i + 1].y * scale + offset_y
//         };

//         // Calculate the direction and distance between points
//         sf::Vector2f direction = end - start;
//         float length = std::sqrt(direction.x * direction.x + direction.y * direction.y);
//         float angle = std::atan2(direction.y, direction.x) * 180.f / M_PI;

//         // Dash size
//         float dashLength = 20.0f;  // Length of each dash
//         float dashSpacing = 10.0f; // Space between dashes
//         float thickness = 3.0f;    // Thickness of the dash

//         // Draw dashes along the line
//         float covered = 0;
//         while (covered < length) {
//             float segmentLength = std::min(dashLength, length - covered);

//             sf::RectangleShape dash(sf::Vector2f(segmentLength, thickness));
//             dash.setFillColor(sf::Color::White);
//             dash.setPosition(start + (direction / length) * covered);
//             dash.setRotation(angle);  // Rotate the dash along the line

//             waypointTexture.draw(dash);

//             covered += segmentLength + dashSpacing;
//         }
//     }
// }



// if(event.type == sf::Event::MouseButtonPressed)
// {
//     if(event.mouseButton.button == sf::Mouse::Left){
//         isDragging= true;
//         lastMousePos = window.mapPixelToCoords(sf::Vector2i(event.mouseButton.x,event.mouseButton.y));
//     }
// }
// if(event.type == sf::Event::MouseMoved && isDragging)
// {
//         sf::Vector2f new_mous_pos = window.mapPixelToCoords(sf::Vector2i(event.mouseButton.x,event.mouseButton.y));
//         sf::Vector2f delta = lastMousePos-new_mous_pos;
//         view.setCenter(view.getCenter()+delta);
//         lastMousePos = new_mous_pos;
//         window.setView(view);
// }
// if(event.type == sf::Event::MouseButtonReleased)
// {
//     if(event.mouseButton.button== sf::Mouse::Left){isDragging=false;}
// }
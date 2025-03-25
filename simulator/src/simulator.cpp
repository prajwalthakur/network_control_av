#include "simulator.hpp"


// simulator constructor 
Simulator::Simulator(int& width,int& height, string& file_name ):window(sf::VideoMode(width,height),"waypoint visualized"){
    // loading waypoints
    load_waypoints(file_name);
    double avg_x = 0.0f;
    double avg_y = 0.0f;
    if(!waypoints.empty()){
        for(const auto& wp:waypoints){
            avg_x+=wp.x;
            avg_y+=wp.y;

        }
        avg_x /= static_cast<double>(waypoints.size());
        avg_y/= static_cast<double>(waypoints.size());
    }

    //scaling and offset to the center of the track
    scale =  20.0f;  
    offset_x = window.getSize().x / 2 - (avg_x * scale);
    offset_y = window.getSize().y / 2 - (avg_y * scale);

    waypointTexture.create(width, height);
    waypointTexture.clear(sf::Color::White);  // Background color

    for(const auto& wp: waypoints){
        sf::CircleShape waypoint(5); //5 pixel ,  5 is radius of circle
        waypoint.setFillColor(sf::Color::Blue);
        // The circle's top-left corner will be placed at (wp.x, wp.y) offset need to make center at wp.x and wp.y
        waypoint.setPosition(wp.x * scale + offset_x -5.0f, wp.y * scale + offset_y-5.0f);   
        waypointTexture.draw(waypoint);
    }
    
    //sprite in SFML is  a drawable object that holds a texture
    //sprite is lighter and faster to draw than individual shapes
    //setTexture() binds this texture to the sprite, so the sprite will now display the texture
    waypointTexture.display();
    waypointSprite.setTexture(waypointTexture.getTexture());

    view.setSize(static_cast<double>(width), static_cast<double>(height));
    view.setCenter(static_cast<double>(width) / 2.f, static_cast<double>(height) / 2.f);
    window.setView(view);
    window.setFramerateLimit(60);   
}

// file_name is the .csv file-name which containts Nx3 entries, first two are (x,y) coordinated
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
        vector<double> v;
        while (std::getline(ss, token, ',')) {
               v.push_back(stof(token));
        }
        waypoints.emplace_back(v[0], v[1]);
    }
    file.close();
    std::cout<<"Loaded "<< waypoints.size()<< " waypoints"<< std::endl;
}

// updates the position of the car object on the track
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
        car.setPosition(car_x* scale + offset_x -4.0f, car_y* scale + offset_y-4.0f);
        window.draw(car);

        // Display frame
        window.display();    
    
    }


void Simulator::run(const double& car_x,const double& car_y, const double& yaw)
    
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
        sf::RectangleShape car(sf::Vector2f(0.58f* scale, 0.38f* scale));
        car.setOrigin(car.getSize().x / 2, car.getSize().y / 2);
        // Set the position, adjusted for scale and offset
        car.setPosition(car_x * scale + offset_x, car_y * scale + offset_y);

        // Convert yaw from radians to degrees (if yaw is already in degrees, you can skip this)
        float angleDegrees = yaw * (180.0f / 3.14f);
        car.setRotation(angleDegrees);

        // Set the fill color
        car.setFillColor(sf::Color::Red);
        window.draw(car);

        // Display frame
        window.display();    
    
    }

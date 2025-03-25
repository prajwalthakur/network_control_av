#pragma once
#include<iostream>
#include <SFML/Graphics.hpp>
#include <vector>
#include <fstream>
#include <sstream>
#include <math.h>
using namespace std;
class Simulator {
    private:
        vector<sf::Vector2f> waypoints;
        sf::RenderTexture waypointTexture;
        sf::Sprite waypointSprite;
        sf::RenderWindow window;  //Window that can serve as a target for 2D drawing
        void load_waypoints(const string&);
        double scale ;
        double offset_x ;
        double offset_y ;
        sf::View view;
        double zoom_factor = 1.0f;
        sf::Vector2f lastMousePos;
        sf::Text Speed_steering_text;
        sf::Font font;
        bool isDragging = false;

    public:
        Simulator(int&, int&, std::string& );
        void run(const double&, const double&);
        void run(const double&, const double&,const double&);
};



Simulator::Simulator(int& width, int& height, string& file_name)
    : window(sf::VideoMode(width, height), "Waypoint Visualized")
    {
    // 1. Load waypoints (only x,y). If your CSV has 3 columns, ignore the third.
    load_waypoints(file_name);

    // 2. Compute the centroid of all waypoints (average x,y).
    double avg_x = 0.0f;
    double avg_y = 0.0f;
    if (!waypoints.empty()) {
        for (const auto& wp : waypoints) {
            avg_x += wp.x;
            avg_y += wp.y;
        }
        avg_x /= static_cast<double>(waypoints.size());
        avg_y /= static_cast<double>(waypoints.size());
    }

    // 3. Decide on a scale factor and offsets to center the waypoints on screen.
    scale =  20.0f;  // Increase or decrease to make the shape bigger or smaller.
    offset_x = window.getSize().x / 2 - (avg_x * scale);
    offset_y = window.getSize().y / 2 - (avg_y * scale);

    // 4. Create an offscreen texture to draw waypoints and dashed lines.
    waypointTexture.create(width, height);
    waypointTexture.clear(sf::Color::Black);  // Background color

    // 5. Draw each waypoint as a circle.
    for (const auto& wp : waypoints) {
        sf::CircleShape waypoint( 4.0f);  // Radius of 5 pixels
        waypoint.setFillColor(sf::Color::White);
        // Subtract 5 so the circle is centered on (wp.x, wp.y).
        double px = wp.x * scale + offset_x - 5.0f;
        double py = wp.y * scale + offset_y - 5.0f;
        waypoint.setPosition(px, py);
        waypointTexture.draw(waypoint);
    }

    // 6. Draw dashed lines between consecutive waypoints.
    if (waypoints.size() > 1) {
        for (size_t i = 0; i < waypoints.size() - 1; ++i) {
            // Convert (x,y) to screen coordinates.
            sf::Vector2f start(
                waypoints[i].x     * scale + offset_x,
                waypoints[i].y     * scale + offset_y
            );
            sf::Vector2f end(
                waypoints[i + 1].x * scale + offset_x,
                waypoints[i + 1].y * scale + offset_y
            );

            // Direction vector, distance, and angle.
            sf::Vector2f direction = end - start;
            double length = std::sqrt(direction.x * direction.x + direction.y * direction.y);
            double angle  = std::atan2(direction.y, direction.x) * 180.f / static_cast<double>(M_PI);

            // Dash parameters (tweak as needed).
            double dashLength  = 20.0f;
            double dashSpacing = 10.0f;
            double thickness   = 3.0f;

            // Draw short segments along the line.
            double covered = 0.0f;
            while (covered < length) {
                double segmentLength = std::min(dashLength, length - covered);
                sf::RectangleShape dash(sf::Vector2f(segmentLength, thickness));
                dash.setFillColor(sf::Color::White);

                // Position the dash at the correct spot along the line.
                dash.setPosition(start + (direction / length) * covered);
                dash.setRotation(angle);  // Rotate the dash to match line orientation

                waypointTexture.draw(dash);
                covered += segmentLength + dashSpacing;
            }
        }
    }

    // 7. Display the texture and create a sprite to render it easily.
    waypointTexture.display();
    waypointSprite.setTexture(waypointTexture.getTexture());

    // 8. Set up the SFML view so everything is visible.
    //    If you don't want to shift it up or down, remove any extra offsets.
    view.setSize(static_cast<double>(width), static_cast<double>(height));
    view.setCenter(static_cast<double>(width) / 2.f, static_cast<double>(height) / 2.f);
    window.setView(view);
    window.setFramerateLimit(60);
}
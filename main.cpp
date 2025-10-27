#include <algorithm>
#include <bits/stdc++.h>
#include <SFML/Graphics.hpp>

using namespace std;

struct Point
{
    double x, y;
    bool operator==(const Point &other) const
    {
        return fabs(x - other.x) < 1e-9 && fabs(y - other.y) < 1e-9;
    }
};

// Compare points by Y (and X if tie) O(1)
bool compareY(const Point &p1, const Point &p2)
{
    if (p1.y == p2.y)
        return p1.x < p2.x;
    return p1.y < p2.y;
}

// Calculate squared distance between two points O(1)
double distanceSq(const Point &a, const Point &b)
{
    return (a.x - b.x) * (a.x - b.x) + (a.y - b.y) * (a.y - b.y);
}

// Determine orientation of ordered triplet (p, q, r) O(1)
// Returns:
// 0 -> collinear
// 1 -> clockwise
// 2 -> counter-clockwise
int orientation(const Point &p, const Point &q, const Point &r)
{
    double val = (q.x - p.x) * (r.y - p.y) -
                 (q.y - p.y) * (r.x - p.x);

    if (fabs(val) < 1e-9)
        return 0; // collinear

    return (val > 0) ? 2 : 1; // CCW -> 2, CW -> 1
}

// Global anchor point for sorting
Point anchor;

// calculate polar angle between anchor and point O(1)
double polarAngle(const Point &p, const Point &anchor = anchor)
{
    return atan2(p.y - anchor.y, p.x - anchor.x);
}

// Sort points by angle only using sort O(n log n)
void sortByAngleAndDistance(vector<pair<Point, double>> &pointAngles)
{
    sort(pointAngles.begin(), pointAngles.end(),
         [](const pair<Point, double> &a, const pair<Point, double> &b)
         {
             if (fabs(a.second - b.second) > 1e-9)
                 return a.second < b.second; // Sort by angle
             // If angles are equal, sort by distance from anchor
             return distanceSq(anchor, a.first) < distanceSq(anchor, b.first);
         });
}

// Step 1: Find the anchor point (lowest Y, then lowest X) O(n)
Point findAnchor(vector<Point> &points)
{
    Point anchorPoint = points[0];
    for (const auto &p : points)
    {
        if (compareY(p, anchorPoint))
            anchorPoint = p;
    }
    return anchorPoint;
}

// Step 2: Sort points by polar angle with respect to anchor O(n log n)
void sortByPolarAngle(vector<Point> &points)
{
    vector<pair<Point, double>> pointAngles;

    for (size_t i = 1; i < points.size(); i++)
    {
        double angle = polarAngle(points[i], anchor);
        pointAngles.push_back({points[i], angle});
    }

    // Sort using merge sort
    sortByAngleAndDistance(pointAngles);

    // Remove points with same angle (keep the farthest one)
    vector<Point> filtered;
    filtered.push_back(points[0]);

    for (size_t i = 0; i < pointAngles.size(); i++)
    {
        // If next angle is same, keep the farthest one
        if (i < pointAngles.size() - 1 && fabs(pointAngles[i].second - pointAngles[i + 1].second) < 1e-9)
        {
            filtered.push_back(pointAngles[i + 1].first);
            i++; // skip next because we already compared
        }
        else
        {
            filtered.push_back(pointAngles[i].first);
        }
    }
    points = filtered;
}

// Graham Scan Algorithm to find Convex Hull O(n log n due to sorting)
vector<Point> grahamScan(vector<Point> points)
{

    // Step 1
    // Find anchor
    anchor = findAnchor(points);

    // Move anchor to front
    int anchorIndex = find(points.begin(), points.end(), anchor) - points.begin();
    swap(points[0], points[anchorIndex]);

    // Step 2
    // Sort by polar angle
    sortByPolarAngle(points);

    // Step 3
    // Use stack to build hull
    stack<Point> hull;
    hull.push(points[0]);
    hull.push(points[1]);

    for (int i = 2; i < points.size(); i++)
    {
        while (hull.size() > 1)
        {
            Point top = hull.top();
            hull.pop();
            Point nextToTop = hull.top();

            if (orientation(nextToTop, top, points[i]) == 2)
            { // counter-clockwise
                hull.push(top);
                break;
            }
        }
        hull.push(points[i]);
    }

    // Convert stack to vector for output
    vector<Point> convexHull;
    while (!hull.empty())
    {
        convexHull.push_back(hull.top());
        hull.pop();
    }
    reverse(convexHull.begin(), convexHull.end());
    return convexHull;
}

int main()
{
    // vector<Point> points = {
    //     {0, 0}, {1, 2}, {2, 1}, {2, 4}, {3, 3}, {4, 0}, {1, 1}};

    vector<Point> points;
    int n;

    do
    {
        cout << "Enter number of points: ";
        cin >> n;

        if (n < 3)
        {
            cout << "Convex hull not possible (less than 3 points)." << endl;
        }

    } while (n < 3);

    cout << "Enter points as x y (space-separated):\n";
    for (int i = 0; i < n; i++)
    {
        Point p;
        cout << "Point " << i + 1 << ": ";
        cin >> p.x >> p.y;
        points.push_back(p);
    }

    vector<Point> hull = grahamScan(points);

    if (hull.empty())
        return 0;

    cout << "Convex Hull Points (in order):\n";
    for (auto &p : hull)
    {
        cout << "(" << p.x << ", " << p.y << ")\n";
    }

    // --- SFML 3.x Window creation ---
    sf::RenderWindow window(sf::VideoMode(sf::Vector2u(600, 500)), "Graham Scan Visualization");
    window.setFramerateLimit(60);

    double minX = numeric_limits<double>::max();
    double maxX = numeric_limits<double>::lowest();
    double minY = numeric_limits<double>::max();
    double maxY = numeric_limits<double>::lowest();

    for (auto &p : points)
    {
        minX = min(minX, p.x);
        maxX = max(maxX, p.x);
        minY = min(minY, p.y);
        maxY = max(maxY, p.y);
    }

    double rangeX = maxX - minX;
    double rangeY = maxY - minY;
    if (rangeX == 0)
        rangeX = 1;
    if (rangeY == 0)
        rangeY = 1;

    const float padding = 50.0f;
    float scaleX = (window.getSize().x - 2 * padding) / rangeX;
    float scaleY = (window.getSize().y - 2 * padding) / rangeY;
    float scale = min(scaleX, scaleY);

    float offsetX = padding - static_cast<float>(minX) * scale;
    float offsetY = window.getSize().y - padding + static_cast<float>(maxY) * scale;

    while (window.isOpen())
    {
        if (auto event = window.pollEvent())
        {
            if (event->is<sf::Event::Closed>())
                window.close();
        }

        window.clear(sf::Color::White);

        // draw all points
        for (auto &p : points)
        {
            sf::CircleShape point(points.size() < 100 ? 5.0f : 2.5f);
            point.setFillColor(sf::Color::Blue);
            point.setOrigin({2.5f, 2.5f});
            point.setPosition({offsetX + static_cast<float>(p.x) * scale,
                               offsetY - static_cast<float>(p.y) * scale});
            window.draw(point);
        }

        // draw hull edges
        for (size_t i = 0; i < hull.size(); i++)
        {
            Point p1 = hull[i];
            Point p2 = hull[(i + 1) % hull.size()];

            std::array<sf::Vertex, 2> line = {
                sf::Vertex{{offsetX + static_cast<float>(p1.x) * scale, offsetY - static_cast<float>(p1.y) * scale}, sf::Color::Red},
                sf::Vertex{{offsetX + static_cast<float>(p2.x) * scale, offsetY - static_cast<float>(p2.y) * scale}, sf::Color::Red}};
            window.draw(line.data(), line.size(), sf::PrimitiveType::Lines);
        }

        window.display();
    }

    return 0;
}
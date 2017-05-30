Point(1) = {0, 0, 0, sz};
Point(2) = {1, 0, 0, sz};
Point(3) = {0, 1, 0, sz};
Point(4) = {1, 1, 0, sz};

Line(1) = {1, 3};
Line(2) = {3, 4};
Line(3) = {4, 2};
Line(4) = {2, 1};

Line Loop(5) = {3, 4, 1, 2};
Plane Surface(6) = {5};

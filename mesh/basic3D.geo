Point(1) = {0, 0, 0, sz};
Point(2) = {1, 0, 0, sz};
Point(3) = {0, 1, 0, sz};
Point(4) = {1, 1, 0, sz};
Point(5) = {0, 0, 1, sz};
Point(6) = {1, 0, 1, sz};
Point(7) = {0, 1, 1, sz};
Point(8) = {1, 1, 1, sz};

Line(1) = {5, 6};
Line(2) = {6, 8};
Line(3) = {8, 7};
Line(4) = {7, 5};
Line(5) = {5, 1};
Line(6) = {1, 3};
Line(7) = {3, 7};
Line(8) = {3, 4};
Line(9) = {4, 8};
Line(10) = {4, 2};
Line(11) = {2, 6};
Line(12) = {2, 1};

Line Loop(13) = {3, 4, 1, 2};
Plane Surface(14) = {13};
Line Loop(15) = {3, -7, 8, 9};
Plane Surface(16) = {15};
Line Loop(17) = {6, 7, 4, 5};
Plane Surface(18) = {17};
Line Loop(19) = {5, -12, 11, -1};
Plane Surface(20) = {19};
Line Loop(21) = {11, 2, -9, 10};
Plane Surface(22) = {21};
Line Loop(23) = {10, 12, 6, 8};
Plane Surface(24) = {23};

Surface Loop(25) = {18, 24, 22, 20, 14, 16};
Volume(26) = {25};

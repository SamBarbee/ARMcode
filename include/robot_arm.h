using namespace vex;

void e_stop();

void inverseKinematicSolve(double x,double y,double z);
void forwardKinematicSolve(double t, double a1, double b1);

void moveJ(double x, double y, double z, double a, double s);
void moveL(double x, double y, double z, double a, double s);

void zero();

void print_xyz();
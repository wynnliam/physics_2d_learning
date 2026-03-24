// Liam Wynn, 2/2/2025, 2D Physics Engine

#include <iostream>
#include "./application.h"

using namespace std;

int main() {
  application app;

  app_setup(app);

  while (app_is_running(app)) {
    app_input(app);
    app_update(app);
    app_draw(app);
  }

  app_destroy(app);

  return 0;
}

#include "Connection.hpp"
#include "BoundingBox.hpp"

Connection::Connection(BoundingBox *const b1, BoundingBox *const b2) {
  double distance = (b1->center() - b2->center()).Length();
  BoundingBox n12(b1,
                  b2);  // TODO: n12 must be reserved for muli-layer level
  attraction_ = b1->radius() * b2->radius() / (distance * distance);
  cost_ = (n12.radius() * n12.radius() * n12.radius()) / attraction_;
}

Connection::~Connection() {}
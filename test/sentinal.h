#pragma once

struct Sentinal {
  int copies = 0;
  int moves = 0;

  Sentinal() = default;

  Sentinal(const Sentinal& x) : copies(x.copies), moves(x.moves) { ++copies; }
  Sentinal(Sentinal&& x) : copies(x.copies), moves(x.moves) { ++moves; }

  Sentinal& operator=(const Sentinal& x) {
    copies = x.copies + 1;
    moves = x.moves;
    return *this;
  }

  Sentinal& operator=(Sentinal&& x) {
    copies = x.copies;
    moves = x.moves + 1;
    return *this;
  }
};

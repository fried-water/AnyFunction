#ifndef SENTINAL_H
#define SENTINAL_H

struct Sentinal {
  int copies = 0;
  int moves = 0;

  Sentinal() = default;

  Sentinal(const Sentinal& x) : copies(x.copies), moves(x.moves) { ++copies; }
  Sentinal(Sentinal&& x) : copies(x.copies), moves(x.moves) { ++moves; }

  Sentinal& operator=(const Sentinal& x) = delete;
  Sentinal& operator=(Sentinal&& x) = delete;
};

#endif
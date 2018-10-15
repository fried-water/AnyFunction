#ifndef SENTINAL_H
#define SENTINAL_H

struct sentinal {
  int copies = 0;
  int moves = 0;

  sentinal() {}

  sentinal(const sentinal& x) : copies(x.copies), moves(x.moves) { ++copies; }
  sentinal(sentinal&& x) : copies(x.copies), moves(x.moves) { ++moves; }

  sentinal& operator=(const sentinal& x) = delete;
  sentinal& operator=(sentinal&& x) = delete;
};

#endif
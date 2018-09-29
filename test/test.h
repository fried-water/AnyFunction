#ifndef TEST_H
#define TEST_H

struct Sentinal {
  int data = 0;

  int copy_con = 0;
  int move_con = 0;
  int copy_ass = 0;
  int move_ass = 0;

  Sentinal(int data) : data(data) {}

  Sentinal(const Sentinal &x)
      : data(x.data), copy_con(x.copy_con), move_con(x.move_con),
        copy_ass(x.copy_ass), move_ass(x.move_ass) {
    std::cout << "Copy: " << data << "\n";
    ++copy_con;
  }

  Sentinal(Sentinal &&x)
      : data(x.data), copy_con(x.copy_con), move_con(x.move_con),
        copy_ass(x.copy_ass), move_ass(x.move_ass) {
    std::cout << "Move: " << data << "\n";
    ++move_con;
  }

  Sentinal &operator=(const Sentinal &x) {
    std::cout << "Copy Ass: " << data << "\n";
    data = x.data;
    copy_con = x.copy_con;
    move_con = x.move_con;
    copy_ass = x.copy_ass + 1;
    move_ass = x.move_ass;
    return *this;
  }

  Sentinal &operator=(Sentinal &&x) {
    std::cout << "Move Ass: " << data << "\n";
    data = x.data;
    copy_con = x.copy_con;
    move_con = x.move_con;
    copy_ass = x.copy_ass;
    move_ass = x.move_ass + 1;
    return *this;
  }
};

void any_function_test();

#endif
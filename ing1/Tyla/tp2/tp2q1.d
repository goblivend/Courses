// TP2Q1.d

import std.stdio;

class Log
{
  private:
    string log_message;
    int day;
    int hour;
    int min;
    int sec;

  public:
    this(string mess, int d, int h, int m, int s)
    {
        log_message = mess;
        day = d;
        hour = h;
        min = m;
        sec = s;
    }

    void set_day(int d)
    {
        day = d;
    }

    void set_hour(int h)
    {
        hour = h;
    }

    void set_min(int m)
    {
        min = m;
    }

    void set_sec(int s)
    {
        sec = s;
    }

    // FIXME-begin
    invariant () {
        assert(1 <= day && day <= 31, "day out of bounds");
        assert(0 <= hour && hour < 24, "hour out of bounds");
        assert(0 <= min && min <= 59, "min out of bounds");
        assert(0 <= sec && sec <= 59, "sec out of bounds");
    }
    // FIXME-end
};

void main()
{
    Log d = new Log("something wrong happened", 5, 5, 30, 42);
    d.set_day(0);   // Fails with error "day out of bounds"
    d.set_hour(35); // Fails with error "hour out of bounds"
    d.set_min(-5);  // Fails with error "min out of bounds"
    d.set_sec(120); // Fails with error "sec out of bounds"
}

// edge detection
/*
  Edge MyEdge = Edge();
  if (MyEdge.rising(digitalRead(0)))
  { }
*/
class Edge
{
  public:
    Edge()
    {
      _boRising = true;
    }

    Edge(bool rising)
    {
      _boRising = rising;
    }

    bool rising(bool state)
    {
      if (state != _boOldState)
      {
        _boOldState = state;

        if (state == true)
        {
          return true;
        }
      }

      return false;
    }

    bool falling(bool state)
    {
      if (state != _boOldState)
      {
        _boOldState = state;

        if (state == false)
        {
          return true;
        }
      }

      return false;
    }

    bool edge(bool state)
    {
      if (state != _boOldState)
      {
        _boOldState = state;
        return true;
      }

      return false;
    }

  private:
    bool _boRising;
    bool _boOldState = false;
};

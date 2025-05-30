final class Point2
{
    int x;
    int y;

    Point2(int x, int y)
    {
        this.x = x;
        this.y = y;
    }

    boolean equals(Point2 a)
    {
        return (this.x == a.x) && (this.y == a.y);
    }

    Point2 add(Point2 a)
    {
        return new Point2(this.x + a.x, this.y + a.y);
    }
}

final class Point2Float
{
    float x;
    float y;

    Point2Float(float x, float y)
    {
        this.x = x;
        this.y = y;
    }

    boolean equals(Point2Float a)
    {
        return (this.x - a.x < 1e-5) && (this.y - a.y < 1e-5);
    }

    Point2Float add(Point2Float a)
    {
        return new Point2Float(this.x + a.x, this.y + a.y);
    }

    Point2Float rotate(float rads)
    {
        return new Point2Float(this.x * cos(rads) - this.y * sin(rads), this.x * sin(rads) + this.y * cos(rads));
    }

    Point2Float scale(float scale)
    {
        return new Point2Float(this.x * scale, this.y * scale);
    }
}

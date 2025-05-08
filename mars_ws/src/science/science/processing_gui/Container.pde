class Container
{
    Point2 position;
    Point2 size;
    int external_padding = 0;
    int internal_padding = 10;
    ArrayList<Container> elements;
    boolean isHorizontal;

    Container(boolean isHorizontal)
    {
        this.isHorizontal = isHorizontal;
        position = new Point2(0,0);
        size = new Point2(external_padding, external_padding);
        elements = new ArrayList<Container>();
    }

    Container(boolean isHorizontal, int external_padding, int internal_padding)
    {
        this(isHorizontal);
        this.external_padding = external_padding;
        this.internal_padding = internal_padding;
        size = new Point2(0, 0);
    }
    
    void update_padding(int external_padding, int internal_padding)
    {
        this.external_padding = external_padding;
        this.internal_padding = internal_padding;
        update();
    }

    void add_container(Container c)
    {
        elements.add(c);
        update();
    }

    void update_position(Point2 new_pos)
    {
        if (position.equals(new_pos)) return;
        position = new_pos;
        update();
    }

    void update_size(Point2 new_size)
    {
        if (size.equals(new_size)) return;
        size = new_size;
        update();
    }

    void update()
    {
        int offset = external_padding;
        int max_size = 0;
        for (int n = 0; n < elements.size(); n++)
        {
            Container cont = elements.get(n);
            cont.update_position(position.add(isHorizontal ? new Point2(offset, external_padding) : new Point2(external_padding, offset)));
            max_size = max(isHorizontal ? cont.size.y : cont.size.x, max_size);
            offset += (isHorizontal ? cont.size.x : cont.size.y) + internal_padding;
        }
        Point2 oldSize = size;
        size = isHorizontal ?
            new Point2(offset - internal_padding + external_padding, max_size + 2 * external_padding)
            : new Point2(max_size + 2 * external_padding, offset - internal_padding + external_padding); 
        //println("Updated size from ", oldSize.x, oldSize.y, " to ", size.x, size.y, " on object ", this);
    }

    void draw()
    {
        noFill();
        stroke(#000000);
        strokeWeight(1);
        rect(position.x, position.y, size.x, size.y);
        for (Container c : elements) c.draw();
    }
}

class ControllerBlock<T> extends Container
{
    Controller<T> controller;

    ControllerBlock(Controller<T> controller)
    {
        super(true);
        size = new Point2(controller.getWidth(), controller.getHeight());
        this.controller = controller;
    }

    void update_position(Point2 new_pos)
    {
        super.update_position(new_pos);
        controller.setPosition(new_pos.x, new_pos.y);
    }

    void update_size(Point2 new_size)
    {
        super.update_size(new_size);
        controller.setSize(new_size.x, new_size.y);
    }
    
    void update()
    {
      // Nothing to update
      return;
    }
}

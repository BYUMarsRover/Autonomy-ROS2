class ScadaView
{
    Poll probeState;
    Poll augerState;
    Poll primaryDoorState;
    Poll secondaryDoorState;
    Poll secondaryCacheState;
    Poll drillControl;

    Poll probeReserved;
    Poll augerReserved;
    Poll primaryCacheDoorReserved;
    Poll secondaryCacheDoorReserved;
    Poll secondaryCacheReserved;

    PShape drill_body;
    PShape[] drill_head;
    PShape linear_actuator_body;
    PShape linear_actuator_arm;
    PShape probe_head;
    PShape cache_body;
    PShape cache_door;
    PShape drill_bit;
    
    int current_drill_head_index = 0;

    LinearActuatorGraphic augerActuator;
    LinearActuatorGraphic probeActuator;
    LinearActuatorGraphic cacheActuator;

    CacheGraphic primaryCache;
    CacheGraphic secondaryCache;

    ProbeHeadGraphic probeHead;
    DrillHeadGraphic drillHead;

    long current_time;
    long prev_time;
    long drill_head_anim_count;

    ScadaView(PollingManager pollingManager)
    {
        getPolls(pollingManager);
        loadShapes();
        loadGraphics();
        reset();
    }

    private Poll fetchPoll(PollingManager pollingManager, String pollName)
    {
        Poll p = pollingManager.getPoll(pollName);
        if (p == null) println("Failed to aquire poll " + pollName + " from polling manager");
        return p;
    }

    private void getPolls(PollingManager pollingManager)
    {
        probeState = fetchPoll(pollingManager, "probe_state");
        augerState = fetchPoll(pollingManager, "auger_state");
        primaryDoorState = fetchPoll(pollingManager, "primary_door_state");
        secondaryDoorState = fetchPoll(pollingManager, "secondary_door_state");
        secondaryCacheState = fetchPoll(pollingManager, "secondary_cache_state");
        drillControl = fetchPoll(pollingManager, "drill_control");

        probeReserved = fetchPoll(pollingManager, "probe_reserved");
        augerReserved = fetchPoll(pollingManager, "auger_reserved");
        primaryCacheDoorReserved = fetchPoll(pollingManager, "primary_cache_door_reserved");
        secondaryCacheDoorReserved = fetchPoll(pollingManager, "secondary_cache_door_reserved");
        secondaryCacheReserved = fetchPoll(pollingManager, "secondary_cache_reserved");
    }

    void tick()
    {
        current_time = millis();
        // float ext = 0.5 * sin((current_time % 1000) * TWO_PI / 1000) + 0.5;

        // Fetch the current positions
        primaryCache.extend(primaryDoorState.value);
        secondaryCache.extend(secondaryDoorState.value);
        probeActuator.extend(probeState.value);
        augerActuator.extend(augerState.value);
        cacheActuator.extend(secondaryCacheState.value);
        
        // Update Drill Animation
        float drill_state = drillControl.value;
        drill_head_anim_count += (current_time - prev_time) * drill_state;
        if (abs(drill_head_anim_count) > 20)
        {
            current_drill_head_index = (current_drill_head_index +
                (drill_head_anim_count > 0 ? 1 : -1)) % 3;
            if (current_drill_head_index < 0) current_drill_head_index = 2; 
            drill_head_anim_count = 0;
        }

        prev_time = current_time;
    };

    void draw()
    {
        probeActuator.draw(probeReserved.value > 0);
        probeHead.draw(probeReserved.value > 0);

        augerActuator.draw(augerReserved.value > 0);
        drillHead.draw(augerReserved.value > 0);

        primaryCache.draw(primaryCacheDoorReserved.value > 0);
        cacheActuator.draw(secondaryCacheReserved.value > 0);
        secondaryCache.draw(secondaryCacheDoorReserved.value > 0);
    };

    private void drawShapeAt(PShape p, float x, float y)
    {
        shape(p, x, y, p.width, p.height);
    };

    private void loadGraphics()
    {
        augerActuator = new LinearActuatorGraphic(100, 50, 0, 0.4);
        drillHead = new DrillHeadGraphic(augerActuator, 0, 0.4);
        primaryCache = new CacheGraphic(drillHead, -2.5*cache_body.width, -2.5*cache_body.height/2, 0, 2.5);

        cacheActuator = new LinearActuatorGraphic(377, 505, PI/2, 0.2);
        secondaryCache = new CacheGraphic(cacheActuator, -2*cache_body.height/2, 2*cache_body.width, -PI/2, 2);

        probeActuator = new LinearActuatorGraphic(450, 170, 0, 0.4);
        probeHead = new ProbeHeadGraphic(probeActuator, 0, 1);
    }

    private void loadShapes()
    {
        drill_body = loadShape("images/drill_body.svg");

        drill_head = new PShape[3];
        drill_head[0] = loadShape("images/drill_head_1.svg");
        drill_head[1] = loadShape("images/drill_head_2.svg");
        drill_head[2] = loadShape("images/drill_head_3.svg");
        current_drill_head_index = 0;
        drill_bit = loadShape("images/drill_bit.svg");

        linear_actuator_body = loadShape("images/linear_actuator_body.svg");
        linear_actuator_arm = loadShape("images/linear_actuator_arm.svg");
        probe_head = loadShape("images/probe_head.svg");
        cache_body = loadShape("images/cache_body.svg");
        cache_door = loadShape("images/cache_door.svg");
    }

    void reset()
    {
        drill_head_anim_count = 0;
        prev_time = millis();
    }

    abstract class Graphic
    {
        float x;
        float y;
        float scale;
        float rotation;
        Graphic mount;

        Graphic(float x, float y, float rotation, float scale)
        {
            this.x = x;
            this.y = y;
            this.rotation = rotation;
            this.scale = scale;
        }

        Graphic(Graphic mount, float x, float y, float rotation, float scale)
        {
            this(x, y, rotation, scale);
            this.mount = mount;
        }

        public void applyTransform()
        {
            translate(x, y);
            rotate(rotation);
            scale(scale);
        }

        public void applyAttachmentTransform()
        {
            Point2Float anchor = getAttachmentPoint();
            translate(anchor.x, anchor.y);
        }

        public void applyFullTransform()
        {
            if (mount != null) mount.applyFullTransform();
            applyTransform();
            applyAttachmentTransform();
        }

        public void draw(boolean highlight)
        {
            pushMatrix();
            if (mount != null) mount.applyFullTransform();
            applyTransform();
            render(highlight);
            popMatrix();
        }

        abstract void render(boolean highlight);

        Point2Float getAttachmentPoint()
        {
            return new Point2Float(x, y);
        }
    }

    class LinearActuatorGraphic extends Graphic
    {
        float extent;

        LinearActuatorGraphic(float x, float y, float rotation, float scale)
        {
            super(x, y, rotation, scale);
            this.extent = 0;
        }

        void extend(float extent)
        {
            this.extent = extent;
        }

        Point2Float getAttachmentPoint()
        {
            float x_val = ((30 + linear_actuator_arm.width/2));
            float y_val = (linear_actuator_arm.height * (1 + 0.9 * extent) + 20);
            return new Point2Float(x_val, y_val);
        }

        Point2Float getRefPoint()
        {
            return new Point2Float(x, y);
        }

        void render(boolean highlight)
        {
            if (highlight)
            {
                fill(255, 0, 0);
                linear_actuator_arm.disableStyle();
                linear_actuator_body.disableStyle();
            }
            drawShapeAt(linear_actuator_arm, 30, linear_actuator_arm.height * 0.9 * this.extent + 20);
            drawShapeAt(linear_actuator_body, 0, 0);
            if (highlight)
            {
                linear_actuator_arm.enableStyle();
                linear_actuator_body.enableStyle();
            }
        }

    }

    class CacheGraphic extends Graphic
    {
        float extent;

        CacheGraphic(float x, float y, float rotation, float scale)
        {
            super(x, y, rotation, scale);
            this.extent = 0;
        }

        CacheGraphic(Graphic mount, float x, float y, float rotation, float scale)
        {
            super(mount, x, y, rotation, scale);
            this.extent = 0;
        }

        void extend(float extent)
        {
            this.extent = extent;
        }

        void render(boolean highlight)
        {
            if (highlight)
            {
                fill(255, 0, 0);
                cache_door.disableStyle();
                cache_body.disableStyle();
            }
            drawShapeAt(cache_door, -cache_door.width * 0.9 * this.extent, cache_body.height * 0.80);
            drawShapeAt(cache_body, 0, 0);
            if (highlight)
            {
                cache_door.enableStyle();
                cache_body.enableStyle();
            }
        }

    }

    class ProbeHeadGraphic extends Graphic
    {
        ProbeHeadGraphic(float x, float y, float rotation, float scale)
        {
            super(x, y, rotation, scale);
        }

        ProbeHeadGraphic(Graphic mount, float rotation, float scale)
        {
            super(mount, 0, 0, rotation, scale);
        }

        void render(boolean highlight)
        {
            if (highlight)
            {
                fill(255, 0, 0);
                probe_head.disableStyle();
            }
            drawShapeAt(probe_head, -(probe_head.width/2), 0);
            if (highlight)
            {
                probe_head.enableStyle();
            }
        }
    }

    class DrillHeadGraphic extends Graphic
    {
        DrillHeadGraphic(float x, float y, float rotation, float scale)
        {
            super(x, y, rotation, scale);
        }

        DrillHeadGraphic(Graphic mount, float rotation, float scale)
        {
            super(mount, 0, 0, rotation, scale);
        }

        Point2Float getAttachmentPoint()
        {
            float x_val = -drill_bit.width/2;
            float y_val = drill_body.height + drill_head[0].height + drill_bit.height*0.2;
            return new Point2Float(x_val, y_val);
        }

        void render(boolean highlight)
        {
            PShape drill_head_selected = drill_head[current_drill_head_index];
            if (highlight)
            {
                fill(255, 0, 0);
                drill_bit.disableStyle();
                drill_head_selected.disableStyle();
                drill_body.disableStyle();
            }
            drawShapeAt(drill_bit, -(drill_bit.width/2), drill_body.height + drill_head_selected.height - 20);
            drawShapeAt(drill_head_selected, -(drill_head_selected.width/2), + (drill_body.height - 20));
            drawShapeAt(drill_body, -(drill_body.width/2), 0);
            if (highlight)
            {
                drill_bit.enableStyle();
                drill_head_selected.enableStyle();
                drill_body.enableStyle();
            }
        }

    }

}

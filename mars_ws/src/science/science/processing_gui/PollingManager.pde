import java.util.HashMap;
import java.util.Map;

class PollingManager
{
    Map<String, Poll> pollMap;

    PollingManager()
    {
        initPolls();
    }

    public void tick()
    {
        long current_time = millis();
        for (String key : pollMap.keySet())
        {
            Poll p = pollMap.get(key);
            if (p.pollingDue(current_time))
                p.execute(current_time);
        }
    }

    public Poll getPoll(String pollName)
    {
        return pollMap.get(pollName);
    }

    private void initPolls()
    {
        pollMap = new HashMap<String, Poll>();

        // Control Polling
        pollMap.put("probe_control",            new UnsignedPoll(sm_interface.author_packet(fm.GetCommand_GetActuatorControl(PROBE_ACTUATOR_INDEX))));
        pollMap.put("auger_control",            new UnsignedPoll(sm_interface.author_packet(fm.GetCommand_GetActuatorControl(AUGER_ACTUATOR_INDEX))));
        pollMap.put("primary_door_control",     new UnsignedPoll(sm_interface.author_packet(fm.GetCommand_GetActuatorControl(PRIMARY_DOOR_ACTUATOR_INDEX))));
        pollMap.put("secondary_door_control",   new UnsignedPoll(sm_interface.author_packet(fm.GetCommand_GetActuatorControl(SECONDARY_DOOR_ACTUATOR_INDEX))));
        pollMap.put("secondary_cache_control",  new UnsignedPoll(sm_interface.author_packet(fm.GetCommand_GetActuatorControl(SECONDARY_CACHE_ACTUATOR_INDEX))));
        pollMap.put("drill_control",            new   SignedPoll(sm_interface.author_packet(fm.GetCommand_GetActuatorControl(DRILL_ACTUATOR_INDEX))));

        getPoll("probe_control").setCriteria(new ScadaCriteria());
        getPoll("auger_control").setCriteria(new ScadaCriteria());
        getPoll("primary_door_control").setCriteria(new ScadaCriteria());
        getPoll("secondary_door_control").setCriteria(new ScadaCriteria());
        getPoll("secondary_cache_control").setCriteria(new ScadaCriteria());
        getPoll("drill_control").setCriteria(new ScadaCriteria());

        // State Polling
        pollMap.put("probe_state",              new UnsignedPoll(sm_interface.author_packet(fm.GetCommand_GetActuatorPosition(PROBE_ACTUATOR_INDEX))));
        pollMap.put("auger_state",              new UnsignedPoll(sm_interface.author_packet(fm.GetCommand_GetActuatorPosition(AUGER_ACTUATOR_INDEX))));
        pollMap.put("primary_door_state",       new UnsignedPoll(sm_interface.author_packet(fm.GetCommand_GetActuatorPosition(PRIMARY_DOOR_ACTUATOR_INDEX))));
        pollMap.put("secondary_door_state",     new UnsignedPoll(sm_interface.author_packet(fm.GetCommand_GetActuatorPosition(SECONDARY_DOOR_ACTUATOR_INDEX))));
        pollMap.put("secondary_cache_state",    new UnsignedPoll(sm_interface.author_packet(fm.GetCommand_GetActuatorPosition(SECONDARY_CACHE_ACTUATOR_INDEX))));

        getPoll("probe_state").setCriteria(new NonZeroControlCriteria(getPoll("probe_control")));
        getPoll("auger_state").setCriteria(new NonZeroControlCriteria(getPoll("auger_control")));
        getPoll("primary_door_state").setCriteria(new NonZeroControlCriteria(getPoll("primary_door_control")));
        getPoll("secondary_door_state").setCriteria(new NonZeroControlCriteria(getPoll("secondary_door_control")));
        getPoll("secondary_cache_state").setCriteria(new NonZeroControlCriteria(getPoll("secondary_cache_control")));

        // Reserved Polling
        pollMap.put("probe_reserved",                   new UnsignedPoll(sm_interface.author_packet(fm.GetCommand_QueryIsActuatorReserved(PROBE_ACTUATOR_INDEX))));
        pollMap.put("auger_reserved",                   new UnsignedPoll(sm_interface.author_packet(fm.GetCommand_QueryIsActuatorReserved(AUGER_ACTUATOR_INDEX))));
        pollMap.put("primary_cache_door_reserved",      new UnsignedPoll(sm_interface.author_packet(fm.GetCommand_QueryIsActuatorReserved(PRIMARY_DOOR_ACTUATOR_INDEX))));
        pollMap.put("secondary_cache_door_reserved",    new UnsignedPoll(sm_interface.author_packet(fm.GetCommand_QueryIsActuatorReserved(SECONDARY_DOOR_ACTUATOR_INDEX))));
        pollMap.put("secondary_cache_reserved",         new UnsignedPoll(sm_interface.author_packet(fm.GetCommand_QueryIsActuatorReserved(SECONDARY_CACHE_ACTUATOR_INDEX))));
        
        getPoll("probe_reserved").setCriteria(new NonZeroControlCriteria(getPoll("probe_reserved")));
        getPoll("auger_reserved").setCriteria(new NonZeroControlCriteria(getPoll("auger_reserved")));
        getPoll("primary_cache_door_reserved").setCriteria(new NonZeroControlCriteria(getPoll("primary_cache_door_reserved")));
        getPoll("secondary_cache_door_reserved").setCriteria(new NonZeroControlCriteria(getPoll("secondary_cache_door_reserved")));
        getPoll("secondary_cache_reserved").setCriteria(new NonZeroControlCriteria(getPoll("secondary_cache_reserved")));
    }

    void addPoll(Poll newPoll, String pollString)
    {
        pollMap.put(pollString, newPoll);
    }
}

enum AttentionLevel
{
    DISABLED,
    PASSIVE,
    ACTIVE
}

class PollActiveCriteria
{
    AttentionLevel getAttentionLevel(){ return AttentionLevel.PASSIVE; }
}

class ManualCriteria extends PollActiveCriteria
{
    AttentionLevel attentionLevel = AttentionLevel.DISABLED;

    AttentionLevel getAttentionLevel()
    {
        return attentionLevel;
    }

    public void setAttentionLevel(AttentionLevel attentionLevel)
    {
        this.attentionLevel = attentionLevel;
    }
}

class ScadaCriteria extends PollActiveCriteria
{
    AttentionLevel getAttentionLevel()
    {
        if (activeTab != 5)
        {
            return AttentionLevel.DISABLED;
        }
        else return AttentionLevel.PASSIVE;
    }
}

class NonZeroControlCriteria extends ScadaCriteria
{
    Poll controlPoll;

    NonZeroControlCriteria(Poll poll)
    {
        super();
        this.controlPoll = poll;
    }

    AttentionLevel getAttentionLevel()
    {
        if (super.getAttentionLevel() == AttentionLevel.DISABLED) { return AttentionLevel.DISABLED; }
        else {
            if (abs(controlPoll.value) > 1e-5) return AttentionLevel.ACTIVE;
            else return AttentionLevel.PASSIVE;
        }
    }
}

abstract class Poll implements Callback
{
    float value;

    long period_active;
    long period_passive;

    AttentionLevel attention;
    PollActiveCriteria criteria;

    long timeout;
    long last_poll_time;
    boolean pending;
    byte[] query;

    Poll(byte[] query, long period_active, long period_passive)
    {
        this.value = 0f;
        this.query = query;
        this.period_active = period_active;
        this.period_passive = period_passive;
        this.last_poll_time = -period_passive;
        this.pending = false;
        this.criteria = null;
    }

    Poll(byte[] query)
    {
        this(query, 100, 1000);
    }

    public Poll setCriteria(PollActiveCriteria criteria)
    {
        this.criteria = criteria;
        return this;
    }

    public AttentionLevel getAttentionLevel()
    {
        return (criteria == null) ? AttentionLevel.PASSIVE : criteria.getAttentionLevel();
    }

    private long getPeriod()
    {
        switch (getAttentionLevel()) {
            case DISABLED:
                return 0;
            case PASSIVE:
                return period_passive;
            case ACTIVE:
                return period_active;
            default:
                return period_passive;
        }
    }

    boolean pollingDue(long currentTime)
    {
        // See if another poll is due
        if (getPeriod() == 0) return false;
        return ((currentTime - last_poll_time > getPeriod()));
    }

    void execute(long currentTime)
    {
        parse.joinCallbackList(this);
        send_serial_bytes(query);
        pending = true;
        last_poll_time = currentTime;
    }

    void receiveModuleResponse(int error_code, byte[] response)
    {
        handleResponse(error_code, response);
        this.pending = false;
    }
    
    byte[] getCallbackSignature()
    {
        return query;
    }

    abstract void handleResponse(int error_code, byte[] response);
}

class SignedPoll extends Poll
{
    SignedPoll(byte[] query, long period_active, long period_passive)
    {
        super(query, period_active, period_passive);
    }

    SignedPoll(byte[] query)
    {
        super(query);
    }

    void handleResponse(int error_code, byte[] response)
    {
        if (response.length == 1)
        {
            int v = int(response[0]);
            while (v < -127) v += 256;
            while (v > 127) v -= 256;
            value = v / 127.0f;
        }
        else println("Unsigned Poll received buffer of size " + response.length + " expected 1");
    }
}

class UnsignedPoll extends Poll
{
    UnsignedPoll(byte[] query, long period_active, long period_passive)
    {
        super(query, period_active, period_passive);
    }

    UnsignedPoll(byte[] query)
    {
        super(query);
    }

    void handleResponse(int error_code, byte[] response)
    {
        if (response.length == 1) value = int(response[0]) / 255.0f;
        else println("Unsigned Poll received buffer of size " + response.length + " expected 1");
    }
}

class SpectorgraphDataPoll extends Poll
{
    SpectrographManipulator parent;
    
    SpectorgraphDataPoll(SpectrographManipulator parent)
    {
        super(sm_interface.author_packet(fm.GetCommand_ReturnSpectrographData()), 100, 100);
        this.parent = parent;
        this.setCriteria(new AwaitingDataControlCriteria(this));
    }

    void handleResponse(int error_code, byte[] response)
    {
        parent.receiveModuleResponse(error_code, response);
    }

    class AwaitingDataControlCriteria extends PollActiveCriteria
    {
        SpectorgraphDataPoll poll;

        AwaitingDataControlCriteria(SpectorgraphDataPoll poll)
        {
            this.poll = poll;
        }

        AttentionLevel getAttentionLevel()
        {
            if (poll.parent.awaiting_data) return AttentionLevel.ACTIVE;
            else return AttentionLevel.DISABLED;
        }
    }
}

class UVDataPoll extends Poll
{
    UVManipulator parent;
    
    UVDataPoll(UVManipulator parent)
    {
        super(parent.getRetrieveDataBytes(), 100, 100);
        this.parent = parent;
        this.setCriteria(new AutoPollEnabledControlCriteria(this));
    }

    void handleResponse(int error_code, byte[] response)
    {
        parent.receiveModuleResponse(error_code, response);
    }

    class AutoPollEnabledControlCriteria extends PollActiveCriteria
    {
        UVDataPoll poll;

        AutoPollEnabledControlCriteria(UVDataPoll poll)
        {
            this.poll = poll;
        }

        AttentionLevel getAttentionLevel()
        {
            if (poll.parent.polling_uv) return AttentionLevel.PASSIVE;
            else return AttentionLevel.DISABLED;
        }
    }
}

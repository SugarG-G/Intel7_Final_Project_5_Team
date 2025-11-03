# Follower Node Readjustment Logic

To view this diagram, copy the text between the "START" and "END" comments below and paste it into a Mermaid-compatible viewer, such as the Mermaid Live Editor (https://mermaid.live).

&lt;!--- MERMAID DIAGRAM START ---&gt;
```mermaid
graph TD
    subgraph External Events
        A[Waypoint 3, 5, or 7 Reached]
        B[Test Service /turn_end Called]
    end

    subgraph "Follower Node Logic"
        C(waypoint_callback)
        D(handle_turn_end)
        E(trigger_retreat)
        F(Pause send_path() by setting is_paused_by_leader_ = true)
        G(Start execute_turn_end_retreat in a new thread)
        H(execute_turn_end_retreat)
        I(Move backward for a calculated time)
        J(Resume send_path() by setting is_paused_by_leader_ = false)
    end

    A --&gt; C
    B --&gt; D
    C --&gt; E
    D --&gt; E
    E --&gt; F
    F --&gt; G
    G -.-> H
    H --&gt; I
    I --&gt; J
```
&lt;!--- MERMAID DIAGRAM END ---&gt;
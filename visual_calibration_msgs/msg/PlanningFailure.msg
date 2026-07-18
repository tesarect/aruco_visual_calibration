# Published once, on the moment a commanded move fails (startup home move,
# cal_ready button, state-sequence return-to-previous/standby moves, etc.)
# — NOT a state topic, so no transient_local/latching: a failure that
# already happened has no meaningful "current value" to replay to a late
# subscriber. Plain reliable QoS, published exactly once per failure.
# Intended consumer: web app shows a popup using message; terminal already
# gets the matching RCLCPP_ERROR independently.

# Short machine-readable label for which operation failed, e.g.
# "startup_home", "cal_ready", "return_to_previous", "standby". Not an
# enum — keep it a plain string so new failure sources don't require a
# .msg change.
string context

# Human-readable failure reason, safe to show directly in a popup.
string message

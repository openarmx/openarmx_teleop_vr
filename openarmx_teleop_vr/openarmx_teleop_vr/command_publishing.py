def publish_active_commands(
    left_publisher,
    right_publisher,
    left_command,
    right_command,
    left_active,
    right_active,
):
    if left_active:
        left_publisher.publish(left_command)
    if right_active:
        right_publisher.publish(right_command)

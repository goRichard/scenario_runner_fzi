def add_title_to_csv(*actor_id_lists):
    title_list = ["timestamp"]
    # for input_list in input_lists:
    for actor_id_list in actor_id_lists:
        for id_number in actor_id_list:
            id_number = str(id_number)
            new_actor_id_list = [
                id_number + "_transform_pitch",
                id_number + "_transform_yaw",
                id_number + "_transform_roll",
                id_number + "_vel_x (m/s)",
                id_number + "_vel_y (m/s)",
                id_number + "_vel_z (m/s)",
                id_number + "_angular_vel_x (rad/s)",
                id_number + "_angular_vel_y (rad/s)",
                id_number + "_angular_vel_z (rad/s)",
                id_number + "_acc_x (m/s\u00b2)",
                id_number + "_acc_y (m/s\u00b2)",
                id_number + "_acc_z (m/s\u00b2) gravity"]
            title_list.extend(new_actor_id_list)

    return title_list


def add_contents_to_csv(tick_time, *actors_lists):
    """

    :param actors: the list of ego and other actors, type ego first
    :param tick_time: every time tick
    :return: list of all the required information from actors
    """
    content_list = [str(tick_time)]
    for actors in actors_lists:
        for actor in actors:
            temp_list = [
                actor.get_transform().rotation.pitch,
                actor.get_transform().rotation.yaw,
                actor.get_transform().rotation.roll,
                actor.get_velocity().x,
                actor.get_velocity().y,
                actor.get_velocity().z,
                actor.get_angular_velocity().x,
                actor.get_angular_velocity().y,
                actor.get_angular_velocity().z,
                actor.get_acceleration().x,
                actor.get_acceleration().y,
                actor.get_acceleration().z
            ]
            content_list.extend([round(temp, 2) for temp in temp_list])

    return content_list


if __name__ == "__main__":
    input_list_1 = ["id_1", "id_2", "id_3"]
    input_list_2 = ["id_4", "id_5", "id_6"]
    output_list = add_title_to_csv(input_list_1, input_list_2)

object_type = " "


def object_type_check(actor_blueprint):
    global object_type
    actor_id = actor_blueprint.id
    motorbike_id = ["vehicle.yamaha.yzf", "vehicle.kawasaki.ninja", "vehicle.harley-davidson.low rider"]
    bife_id = ["vehicle.diamondback.century", "vehicle.bh.crossbike"]
    if actor_id.startswith("vehicle"):
        number_of_wheels = int(actor_blueprint.get_attribute("number_of_wheels"))
        if number_of_wheels == 4:
            if actor_id == "vehicle.carlamotors.carlacola":
                object_type = "truck"
            else:
                object_type = "car"
        elif number_of_wheels == 2:
            if actor_id in motorbike_id:
                object_type = "motorbike"
            elif actor_id in bife_id:
                object_type = "bife"
    elif actor_id.startswith("walker"):
        object_type = "pedestrian"
    return object_type


def add_title(*actor_blueprints_lists):
    global object_type
    title_list = ["timestamp"]
    # for input_list in input_lists:
    for actor_blueprint_list in actor_blueprints_lists:
        for actor_blueprint in actor_blueprint_list:
            object_type = object_type_check(actor_blueprint)
            type_id = actor_blueprint.id
            new_actor_id_list = [
                type_id + "_transform_pitch",
                type_id + "_transform_yaw",
                type_id + "_transform_roll",
                type_id + "_vel_x (m/s)",
                type_id + "_vel_y (m/s)",
                type_id + "_vel_z (m/s)",
                type_id + "_angular_vel_x (rad/s)",
                type_id + "_angular_vel_y (rad/s)",
                type_id + "_angular_vel_z (rad/s)",
                type_id + "_acc_x (m/s\u00b2)",
                type_id + "_acc_y (m/s\u00b2)",
                type_id + "_acc_z (m/s\u00b2) gravity",
                "object_type"]
            title_list.extend(new_actor_id_list)

    return title_list


def add_contents(tick_time, *actors_lists):
    """
    :param actors: the list of ego and other actors, type ego first
    :param tick_time: every time tick
    :return: list of all the required information from actors
    """
    global object_type
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
                actor.get_acceleration().z,
                object_type
            ]
            for i in range(len(temp_list)-1):
                temp_list[i] = round(temp_list[i], 2)
            content_list.extend(temp_list)
    return content_list


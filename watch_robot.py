#!/usr/bin/env python3

import asyncio
import logging
import math
import os
import sys

from pyrevolve import parser
from pyrevolve.custom_logging import logger
from pyrevolve.evolution import fitness
from pyrevolve.revolve_bot import RevolveBot
from pyrevolve.SDF.math import Vector3
from pyrevolve.tol.manage import World
from pyrevolve.util.supervisor.supervisor_multi import DynamicSimSupervisor
from pyrevolve.evolution.fitness import test_fitness as test_fitness 

async def run():
    log = logger.create_logger(
        "experiment",
        handlers=[
            logging.StreamHandler(sys.stdout),
        ],
    )

    # Set debug level to DEBUG
    log.setLevel(logging.DEBUG)

    # Parse command line / file input arguments
    settings = parser.parse_args()

    # Start Simulator
    if settings.simulator_cmd != "debug":
        simulator_supervisor = DynamicSimSupervisor(
            world_file=settings.world,
            simulator_cmd=settings.simulator_cmd,
            simulator_args=["--verbose"],
            plugins_dir_path=os.path.join(".", "build", "lib"),
            models_dir_path=os.path.join(".", "models"),
            simulator_name="gazebo",
        )
    await simulator_supervisor.launch_simulator(port=settings.port_start)
    await asyncio.sleep(0.1)

    # Connect to the simulator and pause
    connection = await World.create(
        settings, world_address=("127.0.0.1", settings.port_start)
    )
    await asyncio.sleep(1)

    # initialization finished

    # load robot file
    path = "phenotype_8657_follow_line.yaml"
    #robot = RevolveBot(_id=settings.test_robot)
    robot = RevolveBot()
    robot.load_file(path, conf_type="yaml")
    robot.update_substrate()

    #robot.save_file(f"{path}.sdf", conf_type="sdf")

    with open('phenotype_8657_follow_line.yaml.sdf', 'r') as file:
        sdf_bot = file.read().replace('\n', '')

    target_direction = 240 / 360 * 2 * math.pi
    target_as_vector = (
        math.cos(target_direction),
        math.sin(target_direction),
        0,
    )
    robot._brain.target = target_as_vector

    robot._brain.target = (0.0 ,10.0 , 0.0)


    # insert robot into the simulator
    #robot_manager = await connection.insert_robot(
    #    robot, Vector3(1, 0, 0.25), life_timeout=None
    #)

    robot_manager = await connection.insert_robot_from_sdf(sdf_bot,
            robot, life_timeout=None
            )

    await asyncio.sleep(2.0)

    # Start the main life loop
    while True:
        # Print robot fitness every second
        status = "dead" if robot_manager.dead else "alive"
        position = robot_manager._positions[-1]
        print(
            f"Robot fitness ({status}) is \n"
            f" OLD:     {fitness.online_old_revolve(robot_manager)}\n"
            f" DISPLAC: {fitness.displacement(robot_manager, robot)}\n"
            f" DIS_VEL: {fitness.displacement_velocity(robot_manager, robot)}\n"
            f" Test Fitness: {test_fitness(robot_manager, robot)}\n",
            f" Robot Position: {position}"
        )
        await asyncio.sleep(2.0)

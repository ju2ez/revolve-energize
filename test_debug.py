import asyncio
import logging
import sys
import os

from pygazebo.connection import DisconnectError
from pyrevolve import parser
from pyrevolve.custom_logging import logger
from pyrevolve.revolve_bot import RevolveBot
from pyrevolve.SDF.math import Vector3
from pyrevolve.tol.manage import World
#from pyrevolve.tol.manage.single_robot_world import SingleRobotWorld as World
from pyrevolve.util.supervisor.supervisor_multi import DynamicSimSupervisor





async def run():
    """
    The main coroutine, which is started below
    """
    print("wytf")
    log = logger.create_logger('experiment', handlers=[
        logging.StreamHandler(sys.stdout),
    ])
    print("wytf")
    # Set debug level to DEBUG
    log.setLevel(logging.DEBUG)

    # Parse command line / file input arguments
    settings = parser.parse_args()

    log.info(f"#### printing plugin path: {os.path.join('.', 'build', 'lib')} ####")

    # Connect to the simulator and pause
    connection = await World.create(settings, world_address=('127.0.0.1', settings.port_start))
    await asyncio.sleep(1)
    await connection.pause(True)
    await connection.reset()

    # initialization finished
    log.info("loading robot")

    # load robot file
    robot = RevolveBot()

    robot_file_path = "phenotype_4798.yaml"        #sven9

    robot.load_file(robot_file_path, conf_type='yaml')
    robot.update_substrate()
    #robot.save_file(f'{robot_file_path}.sdf', conf_type='sdf')

    with open('phenotype_4798.yaml.sdf', 'r') as file:
        sdf_bot = file.read().replace('\n', '')


    # insert robot into the simulator
    robot_manager = await connection.insert_robot_from_sdf(sdf_bot,
                                                           robot, life_timeout=None
                                                           )
    await asyncio.sleep(1.0)
    await connection.pause(False)
    await asyncio.sleep(1.0)

    # Start the main life loop
    while True:
        status = 'dead' if robot_manager.dead else 'alive'
#        best_fitness = None if robot_manager.best_evaluation is None else robot_manager.best_evaluation.fitness
        log.info(f"status: {status} - Robot fitness: ")#{best_fitness}")
        await asyncio.sleep(5.0)



def main():
    def handler(loop, context):
        exc = context['exception']
        if isinstance(exc, DisconnectError) \
                or isinstance(exc, ConnectionResetError):
            print("Got disconnect / connection reset - shutting down.")
            sys.exit(0)
        raise context['exception']

    try:
        loop = asyncio.get_event_loop()
        loop.set_exception_handler(handler)
        loop.run_until_complete(run())
    except KeyboardInterrupt:
        print("Got CtrlC, shutting down.")


if __name__ == '__main__':
    print("STARTING")
    main()
    print("FINISHED")

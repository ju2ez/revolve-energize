//
// Created by matteo on 20/05/2021.
//

#include "Database.h"
#include <sstream>
#include <revolve/utils/Address.h>

using namespace revolve::gazebo;


Database::Database(const std::string &dbname,
                   const std::string &username,
                   const std::string &password,
                   const std::string &address,
                   unsigned int port)
{
    // Translate address in ip address
    ::revolve::utils::Address resolved_address(address, utils::Address::EITHER);

    for (const std::string &addr: resolved_address.get_ips_str()) {
        // Create connection string
        std::ostringstream connection_str;
        connection_str << "dbname = " << dbname
                       << " user = " << username
                       << " hostaddr = " << addr
                       << " port = " << port;
        std::cout << "Trying connection with " << connection_str.str() << std::endl;
        connection_str << " password = " << password;

        postgres = std::make_unique<pqxx::connection>(connection_str.str());
        if (postgres->is_open()) {
            break;
        }
    }

    if (!postgres->is_open()) {
        throw std::runtime_error("Could not open connection to postgresql!");
    } else {
        std::cout << "Connection to Database established" << std::endl;
    }
}

Database::~Database() {
    postgres->close();
}

unsigned long Database::add_robot(const std::string &robot_name) {
    std::ostringstream sql;
    sql << "INSERT INTO robot (id, name) "  \
           "VALUES ( DEFAULT, " << postgres->quote(robot_name) << " ) "
           "RETURNING id;";
    pqxx::work work(*postgres);
    auto result = work.exec(sql.str());
    work.commit();
    unsigned int robot_id = result.at(0)[0].as<unsigned int>();
    return robot_id;
}

pqxx::result Database::add_evaluation(unsigned int robot_id, unsigned int n, double fitness) {
    std::ostringstream sql;
    sql << "INSERT INTO robot_evaluation (robot_id, n, fitness) "
           "VALUES ( " << robot_id << ", " << n << ", " << fitness << ");";
    pqxx::work work(*postgres);
    auto result = work.exec(sql.str());
    work.commit();
    return result;
}

pqxx::result Database::add_state(unsigned int robot_id,
                                 unsigned int eval_id,
                                 const ::gazebo::common::Time& time,
                                 const ignition::math::Pose3d &pose)
{
    std::ostringstream sql;
    const ignition::math::Vector3d &position = pose.Pos();
    const ignition::math::Quaterniond &rot = pose.Rot();
    sql << "INSERT INTO robot_state ( "
           "time_sec,time_nsec,evaluation_n,evaluation_robot_id,"
           "pos_x,pos_y,pos_z,"
           "rot_quaternion_x,rot_quaternion_y,rot_quaternion_z,rot_quaternion_w,"
           "orientation_forward,orientation_left,orientation_back,orientation_right"
           " ) "
           "VALUES ("
           << time.sec << ',' << time.nsec << ',' << eval_id << ',' << robot_id << ','
           << position.X() << ',' << position.Y() << ',' << position.Z() << ','
           << rot.X() << ',' << rot.Y() << ',' << rot.Z() << ',' << rot.W() << ','
           << "0,0,0,0"
           << " ); ";
    auto result = pending_work->exec(sql.str());
    // do not commit every single state, buffer them instead in the `pending_work` :)
    return result;
}

void Database::start_work() {
    assert(!pending_work);
    pending_work = std::make_unique<pqxx::work>(*postgres);
}

void Database::commit() {
    assert(pending_work);
    pending_work->commit();
    pending_work.reset();
}
// Isolated rigid-body contact fixture. No controller or pose overrides after launch.
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/physics/ContactManager.hh>
#include <fstream>
#include <iomanip>

namespace gazebo {
class SingleTrunkProbe : public ModelPlugin {
  physics::ModelPtr model;
  physics::WorldPtr world;
  event::ConnectionPtr connection;
  std::ofstream states, contacts;
public:
  void Load(physics::ModelPtr m, sdf::ElementPtr sdf) override {
    model=m; world=m->GetWorld();
    states.open(sdf->Get<std::string>("states"));
    contacts.open(sdf->Get<std::string>("contacts"));
    if (!states || !contacts) gzthrow("Cannot write single-trunk probe evidence");
    states << "stamp,x,y,z,vx,vy,vz,roll,pitch,yaw\n" << std::setprecision(17);
    contacts << "stamp,collision1,collision2,x,y,z,depth,force_n\n" << std::setprecision(17);
    world->Physics()->GetContactManager()->SetNeverDropContacts(true);
    model->SetLinearVel(ignition::math::Vector3d(sdf->Get<double>("speed"),0,0));
    connection=event::Events::ConnectWorldUpdateEnd(std::bind(&SingleTrunkProbe::Tick,this));
  }
  void Tick() {
    const double t=world->SimTime().Double();
    auto p=model->WorldPose(); auto v=model->WorldLinearVel(); auto e=p.Rot().Euler();
    states << t << ',' << p.Pos().X() << ',' << p.Pos().Y() << ',' << p.Pos().Z()
           << ',' << v.X() << ',' << v.Y() << ',' << v.Z()
           << ',' << e.X() << ',' << e.Y() << ',' << e.Z() << '\n';
    auto manager=world->Physics()->GetContactManager();
    for (unsigned i=0;i<manager->GetContactCount();++i) {
      auto c=manager->GetContact(i);
      if (!c || !c->collision1 || !c->collision2) continue;
      auto n1=c->collision1->GetScopedName(), n2=c->collision2->GetScopedName();
      if (n1.find("single_trunk")==std::string::npos && n2.find("single_trunk")==std::string::npos) continue;
      for (int j=0;j<c->count;++j) {
        const auto p=c->positions[j];
        contacts << t << ',' << n1 << ',' << n2 << ',' << p.X() << ',' << p.Y() << ',' << p.Z()
                 << ',' << c->depths[j] << ',' << c->wrench[j].body1Force.Length() << '\n';
      }
    }
    states.flush(); contacts.flush();
  }
};
GZ_REGISTER_MODEL_PLUGIN(SingleTrunkProbe)
}

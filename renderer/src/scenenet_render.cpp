/*
 * This file is part of SceneNet RGB-D.
 *
 * Copyright (C) 2017 Imperial College London
 * 
 * The use of the code within this file and all code within files that 
 * make up the software that is SemanticFusion is permitted for 
 * non-commercial purposes only.  The full terms and conditions that 
 * apply to the code within this file are detailed within the LICENSE.txt 
 * file and at <http://www.imperial.ac.uk/dyson-robotics-lab/downloads/semantic-fusion/scenenet-rgbd-license/> 
 * unless explicitly stated.  By downloading this file you agree to 
 * comply with these terms.
 *
 * If you wish to use any of this code for commercial purposes then 
 * please email researchcontracts.engineering@imperial.ac.uk.
 *
 */

#include "Renderer/OptixRenderer.h"
#include "Scene/SceneNet.h"
#include "Util/sutil/sutil.h"

#include <iostream>
#include <memory>
#include <string>
#include <unistd.h>
#include <signal.h>

#include <boost/filesystem.hpp>

#include "cereal/archives/json.hpp"

namespace bf = boost::filesystem;

class BaseScene {
public:
  BaseScene() : m_render_type(RenderType::IMAGE) {}
  void   initScene(std::string save_base,std::string layout_file,std::string obj_base_folder,std::string layout_base_folder,int gpu);
  bool   trace(std::string save_name, int frame_num);
  optix::Buffer getOutputBuffer();
  optix::Buffer getRawOutputBuffer();

private:
  std::unique_ptr<OptixRenderer> m_renderer;
  std::unique_ptr<SceneNet>      m_iscene;
  RenderType      m_render_type;

  static unsigned int WIDTH;
  static unsigned int HEIGHT;
};

unsigned int BaseScene::WIDTH  = 320u;
unsigned int BaseScene::HEIGHT = 240u;

void BaseScene::initScene(std::string save_base,std::string layout_file,std::string obj_base_folder,std::string layout_base_folder, int gpu) {
  // A seed of 0 signifies a random seed - any other number means to use that as
  // the deterministic seed value
  int seed = 0;
  m_iscene.reset(new SceneNet(save_base,layout_file,obj_base_folder,layout_base_folder,seed));
  //Check the scene isn't too bright or dark (i.e. bleached or totally black,with quick render
  m_renderer.reset(new OptixRenderer(WIDTH,HEIGHT,false));
  m_renderer->initialize(gpu,1,1);
  m_renderer->initScene(*m_iscene);
  m_renderer->calculatePhotonMap();
  const SceneNetCamera& default_camera = m_iscene->getCamera();
  const std::pair<TooN::Vector<3>,TooN::Vector<3> > start_pose = m_iscene->getPose();
  if (start_pose.first[0] == 0.0 && start_pose.first[1] == 0.0 && start_pose.first[2] == 0.0 &&
      start_pose.second[0] == 0.0 && start_pose.second[1] == 0.0 && start_pose.second[2] == 0.0) {
    exit(1);
  }
  const std::pair<TooN::Vector<3>,TooN::Vector<3> > end_pose = m_iscene->getPose();
  if (end_pose.first[0] == 0.0 && end_pose.first[1] == 0.0 && end_pose.first[2] == 0.0 &&
      end_pose.second[0] == 0.0 && end_pose.second[1] == 0.0 && end_pose.second[2] == 0.0) {
    exit(1);
  }

  // CHECK for average intensity of the scene... 
  m_renderer->render(default_camera,start_pose,end_pose,RenderType::IMAGE);
  double average_intensity = 0.0;
  optix::uchar4* buffer_Host = (optix::uchar4*) m_renderer->getOutputBuffer(RenderType::IMAGE)->map();
  for(int i = 0; i < WIDTH * HEIGHT; ++i) {
    optix::uchar4 rgb_val = buffer_Host[i];
    int max_val = 0;
    max_val = std::max(max_val,static_cast<int>(rgb_val.x));
    max_val = std::max(max_val,static_cast<int>(rgb_val.y));
    max_val = std::max(max_val,static_cast<int>(rgb_val.z));
    average_intensity += max_val;
  }
  average_intensity /= WIDTH * HEIGHT;
  m_renderer->getOutputBuffer(RenderType::IMAGE)->unmap();
  std::cout<<"Average intensity:"<<average_intensity<<std::endl;
  if (average_intensity < 60 || average_intensity > 220) {
    std::cout<<"Average intensity too extreme:"<<average_intensity<<std::endl;
    exit(1);
  }
  // Use this for normal quality renders
  // Samples are squared so 4 means 16 total
  m_renderer->setNumIterations(4);//4
  m_renderer->setNumPhotonMaps(4);//4
  m_renderer->calculatePhotonMap();
}

optix::Buffer BaseScene::getOutputBuffer() {
  return m_renderer->getOutputBuffer(m_render_type);
}

optix::Buffer BaseScene::getRawOutputBuffer() {
  return m_renderer->getRawOutputBuffer(m_render_type);
}

bool BaseScene::trace(std::string save_name_base, int frame_num) {
  const SceneNetCamera& default_camera = m_iscene->getCamera();
  bool reset = false;
  if (frame_num == 0) {
    reset=true;
  }

  // The two poses are for motion blur
  const std::pair<TooN::Vector<3>,TooN::Vector<3> > start_pose = m_iscene->getPose(reset,false);
  if (start_pose.first[0] == 0.0 && start_pose.first[1] == 0.0 && start_pose.first[2] == 0.0 &&
      start_pose.second[0] == 0.0 && start_pose.second[1] == 0.0 && start_pose.second[2] == 0.0) {
    return false;
  }
  const std::pair<TooN::Vector<3>,TooN::Vector<3> > end_pose = m_iscene->getPose(false,reset);
  if (end_pose.first[0] == 0.0 && end_pose.first[1] == 0.0 && end_pose.first[2] == 0.0 &&
      end_pose.second[0] == 0.0 && end_pose.second[1] == 0.0 && end_pose.second[2] == 0.0) {
    return false;
  }

  // TODO: this should be configurable
  if (frame_num % 12 != 0) {
    return true;
  }

  char fnum[10];
  sprintf(fnum, "%05d", frame_num);
  std::string fnum_str(fnum);
  
  std::cout<<"About to render RGB"<<std::endl;
  m_render_type = RenderType::IMAGE;
  m_renderer->render(default_camera,start_pose,end_pose,m_render_type);
  std::string save_name = save_name_base+fnum_str+"_rgb.jpg";
  std::cout<<"Saving here:"<<save_name<<std::endl;

  //std::cout<<"Saving file:"<<save_name<<std::endl;
  sutil::displayBufferPPM(save_name.c_str(),getOutputBuffer()->get());
  // Also save ground truth
  // Only need to call render once for all ground truth
  m_render_type = RenderType::INSTANCE;
  m_renderer->render(default_camera,start_pose,end_pose,m_render_type);
  save_name = save_name_base+fnum_str+"_instance.png";
  std::cout<<"Instance saving here:"<<save_name<<std::endl;
  sutil::displayBufferPPM(save_name.c_str(),getRawOutputBuffer()->get());

  m_render_type = RenderType::DEPTH;
  save_name = save_name_base+fnum_str+"_depth.png";
  std::cout<<"Depth saving here:"<<save_name<<std::endl;
  sutil::displayBufferPPM(save_name.c_str(),getRawOutputBuffer()->get());

  return true;
}

struct status_t
{
  bool done;
  int step;
  // materials, et al?

  status_t()
    : done(false), step(0)
  {    
  }
  
  template <class Archive>
  void serialize(Archive& ar) {
    ar(done, step);
  }
};

status_t load_status(bf::path& file)
{
  std::ifstream in(file.string());
  cereal::JSONInputArchive archive(in);
  status_t s;
  archive(s);
  return s;
}

void save_status(bf::path& file, status_t& status)
{
  std::ofstream out(file.string());
  cereal::JSONOutputArchive archive(out);
  archive(status);
}

volatile sig_atomic_t g_interrupted = 0;

void term(int sig)
{
  std::cout << "TERM HANDLER CALLED" <<std::endl;
  g_interrupted = 1;
}

int main(int argc, char* argv[]) {
  // install the signal handlers
  struct sigaction action;
  memset(&action, 0, sizeof(struct sigaction));
  action.sa_handler = term;
  sigaction(SIGTERM, &action, NULL);
  sigaction(SIGINT, &action, NULL);
  sigaction(SIGALRM, &action, NULL);

  // set an alarm for one hour (in seconds)
  alarm(3600);
  
  if (argc < 3) {
    std::cout<<"Too few arguments"<<std::endl;
    std::cout<<"./render /path/to/ShapeNet/ /path/to/SceneNetLayouts/ /path/to/scene_and_trajectory_description.txt"<<std::endl;
    std::cout<<"Note that the folders should be followed by trailing / in the command"<<std::endl;
    exit(1);
  }
  std::string shapenets_dir = std::string(argv[1]);
  std::cout<<"ShapeNet directory:"<<shapenets_dir<<std::endl;
  std::string layouts_dir = std::string(argv[2]);
  std::cout<<"Layouts directory:"<<layouts_dir<<std::endl;
  std::string scene_description_file = std::string(argv[3]);
  std::cout<<"Input Scene Description:"<<scene_description_file<<std::endl;
  bf::path sdf(scene_description_file);
  bf::path output_dir = sdf.parent_path() / "data/";
  bf::path status_file = output_dir / "status.txt";
  status_t status;
    
  if (argc >= 5) {
    output_dir = std::string(argv[4]);
  }

  if (!bf::exists(output_dir)) {
    std::string od = output_dir.string();
    if (output_dir.filename() == ".") {
      od = output_dir.parent_path().string();
    }
    if (!bf::create_directories(od)) {
      std::cerr << "Problem creating dir: " << output_dir.string() << std::endl;
      exit(2);
    }
  } else {
    // if the output directory exists, check for a status object
    if (bf::exists(status_file)) {
      // load the status
      status = load_status(status_file);
      if (status.done) {
        // do nothing...
        return 0;
      }
    }
  }    
    
  BaseScene scene;
  scene.initScene(output_dir.string(),scene_description_file,shapenets_dir,layouts_dir,0);
  const int number_trajectory_steps = 10000;
  for (int i = status.step; i < number_trajectory_steps; ++i) {
    save_status(status_file, status);
    if (g_interrupted == 1) return 0; // exit CLEANLY, but not done yet
    if (!scene.trace(output_dir.string(),i)) {
      std::cout<<"Finished render"<<std::endl;
      break;
    }
    status.step++;
  }
  status.done = true;
  save_status(status_file, status);
    
  return 0;
}

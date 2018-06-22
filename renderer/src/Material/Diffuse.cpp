/* 
 * Copyright (c) 2013 Opposite Renderer
 * For the full copyright and license information, please view the LICENSE.txt
 * file that was distributed with this source code.
 */

#include "Diffuse.h"
#include "Renderer/RayType.h"
#include <iostream>

bool Diffuse::m_optixMaterialIsCreated = false;
optix::Material Diffuse::m_optixMaterial;

Diffuse::Diffuse(const Vector3 & Kd, int class_id, int instance_id)
  : Material(class_id,instance_id)
{
    this->Kd = Kd;
}

optix::Material Diffuse::getOptixMaterial(optix::Context & context)
{
    if(!m_optixMaterialIsCreated)
    {
        m_optixMaterial = context->createMaterial();
        std::string ptx = ptxpath() + "Diffuse.cu.ptx";
        std::cout << "Loading ptx: " << ptx << std::endl;
        optix::Program radianceProgram = context->createProgramFromPTXFile(ptx, "closestHitRadiance");
        optix::Program photonProgram = context->createProgramFromPTXFile(ptx, "closestHitPhoton");
        m_optixMaterial->setClosestHitProgram(RayType::RADIANCE, radianceProgram);
        m_optixMaterial->setClosestHitProgram(RayType::RADIANCE_IN_PARTICIPATING_MEDIUM, radianceProgram);
        m_optixMaterial->setClosestHitProgram(RayType::PHOTON, photonProgram);
        m_optixMaterial->setClosestHitProgram(RayType::PHOTON_IN_PARTICIPATING_MEDIUM, photonProgram);
        m_optixMaterial->validate();

        this->registerMaterialWithShadowProgram(context, m_optixMaterial);
        this->registerMaterialWithGroundTruthGenerators(context, m_optixMaterial);
        m_optixMaterialIsCreated = true;
    }
    return m_optixMaterial;
}

/*
// Register any material-dependent values to be available in the optix program.
*/

void Diffuse::registerGeometryInstanceValues(optix::GeometryInstance & instance )
{
    instance["Kd"]->setFloat(this->Kd);
}

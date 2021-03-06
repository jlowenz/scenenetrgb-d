/* 
 * Copyright (c) 2013 Opposite Renderer
 * For the full copyright and license information, please view the LICENSE.txt
 * file that was distributed with this source code.
*/

#pragma once
namespace RayType
{
    enum E
    {
        RADIANCE,
        SHADOW,
        PHOTON,
        SHADOW_PHOTON,
        RADIANCE_IN_PARTICIPATING_MEDIUM,
        PHOTON_IN_PARTICIPATING_MEDIUM,
        VOLUMETRIC_RADIANCE,
        PARTICIPATING_MEDIUM_TRANSMISSION,
        GROUND_TRUTH,
        NUM_RAY_TYPES
    };
}

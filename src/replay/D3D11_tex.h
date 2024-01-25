/*
 * Copyright (c) 2022, NVIDIA CORPORATION.  All rights reserved.
 *
 * NVIDIA CORPORATION and its licensors retain all intellectual property
 * and proprietary rights in and to this software, related documentation
 * and any modifications thereto.  Any use, reproduction, disclosure or
 * distribution of this software and related documentation without an express
 * license agreement from NVIDIA CORPORATION is strictly prohibited.
 */


#pragma once

#include <d3d11.h>

#include <fstream>

#define VENDORID_NVIDIA  0x10DE

struct TextureSet
{
    void Release()
    {
        for (int i = 0; i < NUM_TEX; i++)
        {
            m_tex[i]->Release();
        }
    }
    static const int NUM_TEX = 3;
    ID3D11Texture2D* m_tex[NUM_TEX];
    HANDLE m_sharedHandles[NUM_TEX];
    DXGI_FORMAT m_textureFormat;
    int m_srcIdx;
    int m_nextIdx;
    int m_width;
    int m_height;
    uint32_t m_pid;
};


class RenderHelper {
    IDXGIFactory1* m_dxgiFactory1{ nullptr };
    ID3D11Device* m_d3dDevice{ nullptr };
    ID3D11DeviceContext* m_immContext{ nullptr };

public:
    bool hasNVGPU()
    {
        // Init DX
        CreateDXGIFactory1(__uuidof(IDXGIFactory1), (void**)&m_dxgiFactory1);
        if (!m_dxgiFactory1)
        {
            return false;
        }
        IDXGIAdapter* pAdapter = nullptr;
        DXGI_ADAPTER_DESC adapterDesc;

        // look for NV GPU
        for (UINT iAdapter = 0; m_dxgiFactory1->EnumAdapters(iAdapter, &pAdapter) != DXGI_ERROR_NOT_FOUND; ++iAdapter)
        {
            pAdapter->GetDesc(&adapterDesc);
            if (adapterDesc.VendorId == VENDORID_NVIDIA)
            {
                return true;
            }
        }

        return false;
    }

    bool Init()
    {
        if (m_d3dDevice)
        {
            return true;
        }

        HRESULT hr = NULL;

        // Init DX
        CreateDXGIFactory1(__uuidof(IDXGIFactory1), (void**)&m_dxgiFactory1);
        if (!m_dxgiFactory1)
        {
            return false;
        }

        IDXGIAdapter* pAdapter = nullptr;
        DXGI_ADAPTER_DESC adapterDesc;

        for (UINT iAdapter = 0; m_dxgiFactory1->EnumAdapters(iAdapter, &pAdapter) != DXGI_ERROR_NOT_FOUND; ++iAdapter)
        {
            pAdapter->GetDesc(&adapterDesc);
            if (adapterDesc.VendorId == VENDORID_NVIDIA)
            {
                break;
            }
        }

        if (!pAdapter)
        {
            return false;
        }

        D3D_FEATURE_LEVEL featureLevels[] =
        {
            D3D_FEATURE_LEVEL_11_0,
            D3D_FEATURE_LEVEL_10_1,
            D3D_FEATURE_LEVEL_10_0,
        };
        UINT numFeatureLevels = ARRAYSIZE(featureLevels);

#ifdef _DEBUG // attempt to create a debug device (not all systems have the libraries installed)
        //hr = D3D11CreateDevice(pAdapter, pAdapter ? D3D_DRIVER_TYPE_UNKNOWN : D3D_DRIVER_TYPE_HARDWARE, NULL, D3D11_CREATE_DEVICE_DEBUG, featureLevels, numFeatureLevels, D3D11_SDK_VERSION, &m_d3dDevice, NULL, &m_immContext);
#endif

        if (!m_d3dDevice)
        {
            hr = D3D11CreateDevice(pAdapter, pAdapter ? D3D_DRIVER_TYPE_UNKNOWN : D3D_DRIVER_TYPE_HARDWARE, NULL, 0, featureLevels, numFeatureLevels, D3D11_SDK_VERSION, &m_d3dDevice, NULL, &m_immContext);
        }

        if (FAILED(hr))
        {
            return false;
        }
        IDXGIDevice1* DXGIDevice1 = nullptr;
        m_d3dDevice->QueryInterface(__uuidof(IDXGIDevice1), (void**)&DXGIDevice1);

        if (DXGIDevice1)
        {
            if (FAILED(hr = DXGIDevice1->SetMaximumFrameLatency(1)))
            {
            }
            DXGIDevice1->Release();
        }

        return true;
    }

    ID3D11Texture2D* CreateTexture(int width, int height, UINT miscFlags, DXGI_FORMAT format)
    {
        auto texFormat = format;

        ID3D11Texture2D* tex;
        D3D11_TEXTURE2D_DESC desc;

        memset(&desc, 0, sizeof(desc));
        desc.Width = width;
        desc.Height = height;
        desc.Format = texFormat;
        desc.MipLevels = desc.ArraySize = 1;
        desc.SampleDesc.Count = 1;
        desc.SampleDesc.Quality = 0;
        desc.Usage = D3D11_USAGE_DEFAULT;
        desc.BindFlags = D3D11_BIND_SHADER_RESOURCE | D3D11_BIND_RENDER_TARGET;
        desc.CPUAccessFlags = 0;
        desc.MiscFlags = miscFlags;

        HRESULT hr;

        if ((hr = m_d3dDevice->CreateTexture2D(&desc, NULL, &tex)) < 0)
        {
            return NULL;
        }
        return tex;
    }


};
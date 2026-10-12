#pragma once
#include <Siv3D.hpp>

void SDNormalReconstruct()
{
	Image inputImage{ U"example/ship_01.jpg" };
	Image L = inputImage;
	Image resultImage = inputImage;
	DynamicTexture displayTexture{ inputImage };
	DynamicTexture resultTexture{ inputImage };
	const int ImageWidth = inputImage.width();
	const int ImageHeight = inputImage.height();

	Array<Array<double>> depthLayer;
	depthLayer.resize(ImageHeight);
	for (int i = 0; i < depthLayer.size(); i++) { depthLayer[i].resize(ImageWidth, 0.0); }
	auto inputLayer = depthLayer;

	Array<Array<Vec3>> normals;
	normals.resize(ImageHeight);
	for (int i = 0; i < normals.size(); i++) { normals[i].resize(ImageWidth, Vec3::Zero()); }

	Vec3 pointLight = { 100, 100, 2.0 };

	auto f = [](const double& x)
		{
			return (3.0 + (-6.0 + 4.0 * x) * x) * x;
		};

	// Sobel Util
	const Array<Array<double>> weightsY{
	{-2,-2,-4,-2,-2},
	{-1,-1,-2,-1,-1},
	{0,0,0,0,0},
	{1,1,2,1,1},
	{2,2,4,2,2}
	};
	const Array<Array<double>> weightsX{
	{-2,-1,0,1,2},
	{-2,-1,0,1,2},
	{-4,-2,0,2,4},
	{-2,-1,0,1,2},
	{-2,-1,0,1,2}
	};

	auto SobelFilter = [&](const Array<Array<double>>& p, const Array<Array<double>>& weights)
		{
			Array<Array<double>> result;
			result.resize(ImageHeight);
			for (int i = 0; i < result.size(); i++) { result[i].resize(ImageWidth, 0.0); }

			for (int i = 0; i < ImageWidth; i++)
			{
				for (int j = 0; j < ImageHeight; j++)
				{
					double sum = 0.0;
					for (int w = -2; w <= 2; w++)
					{
						for (int h = -2; h <= 2; h++)
						{
							const int width = Clamp(i + w, 0, ImageWidth - 1);
							const int height = Clamp(j + h, 0, ImageHeight - 1);
							const double weight = weights[h + 2][w + 2];
							sum += p[height][width] * weight;
						}
					}

					result[j][i] = sum;
				}
			}

			return result;
		};

	// ImageConvert Util
	auto FillImageFromArrayDouble = [&](const Array<Array<double>>& src)
		{
			auto s = inputImage;
			for (int i = 0; i < ImageWidth; i++)
			{
				for (int j = 0; j < ImageHeight; j++)
				{
					s[j][i] = ColorF{ Abs(src[j][i]), 0.0, 0.0, 1.0 };
				}
			}
			displayTexture.fill(s);
		};

	auto FillImageFromArrayVec3 = [&](const Array<Array<Vec3>>& src)
		{
			auto s = inputImage;
			for (int i = 0; i < ImageWidth; i++)
			{
				for (int j = 0; j < ImageHeight; j++)
				{
					Vec3 r = src[j][i] * 0.5 + Vec3::One() * 0.5;
					s[j][i] = ColorF{ r.x, r.y, r.z, 1.0 };
				}
			}
			displayTexture.fill(s);
		};

	auto RecoverNormalProcess = [&]()
		{
			auto r = inputImage;

			// AlphaをRに変換 
			for (int i = 0; i < ImageWidth; i++)
			{
				for (int j = 0; j < ImageHeight; j++)
				{
					r[j][i] = ColorF{ static_cast<double>(inputImage[j][i].a), 0.0, 0.0, 1.0 };

					// YCrCbのYを計算,その後でdepth=Yとしてしまう
					Vec3 color = Vec3{ inputImage[j][i].r,inputImage[j][i].g,inputImage[j][i].b } / 255.0;
					double Y = Dot(color, Vec3{ 0.213,0.715,0.072 });
					L[j][i] = ColorF{ Y,0.0,0.0,1.0 };

					// 保持しておく
					inputLayer[j][i] = Y;
				}
			}

			// Bilateral Filterを適用してぼかす
			constexpr double SIGMA_COLOR = 0.1;
			constexpr double SIGMA_SPACE = 10.0;
			L.bilateralFilter(-1, SIGMA_COLOR, SIGMA_SPACE);
		};

	double baseWeight = 1.0; double detailWeight = 0.9;

	auto ChangeBaseValue = [&]()
		{
			// 後々の計算を楽にするためArrayで変換
			for (int i = 0; i < ImageWidth; i++)
			{
				for (int j = 0; j < ImageHeight; j++)
				{
					// Base層において、暗部は深くまた明るい部分も深いというのを実現したい
					// そのためにGradient Reshape Functionで平滑化しておく
					// (Lumoの平滑化処理に着想)
					double base = f(f(f(f(L[j][i].r / 255.0))));

					// Two-scale Tone Management for Photographic Look参照
					// DetailをD=L-Bで定義
					double detail = inputLayer[j][i] - base;

					depthLayer[j][i] = base * baseWeight + detail * detailWeight;
				}
			}
		};

	auto ReconstrcutNormalProcess = [&]()
		{
			auto nablaX = SobelFilter(depthLayer, weightsX);
			auto nablaY = SobelFilter(depthLayer, weightsY);

			for (int i = 0; i < ImageWidth; i++)
			{
				for (int j = 0; j < ImageHeight; j++)
				{
					Vec3 gx = {1.0, 0.0, nablaX[j][i]};
					Vec3 gy = {0.0, 1.0, nablaY[j][i]};
					Vec3 r = Math::Cross(gx, gy).normalized();
					r = r * 0.5 + Vec3::One() * 0.5;
					resultImage[j][i] = ColorF(r.x, r.y, r.z, 1.0);
				}
			}

			resultTexture.fill(resultImage);
		};

	RecoverNormalProcess();
	ChangeBaseValue();
	ReconstrcutNormalProcess();

	Array<String> texts{ U"Input", U"Depth", U"Normal", U"Lighting" };
	size_t prevIndex = 0;
	size_t index = 0;

	struct ShaderParam
	{
		Vector3D<float> m_lightColor = { 1.0f,1.0f,1.0f };
		Vector3D<float> m_lightDirection = { 0.0f,0.0f,1.0f };
	};

	const PixelShader pixelShader = HLSL{ U"example/shader/hlsl/sd_lighting.hlsl", U"PS" };
	ConstantBuffer<ShaderParam> cb;

	HSV lightColor = { 1,1,1 };
	Vec3 lightDir = { 0.0,0.0,1.0 };

	bool isForceUpdate = false;
	bool isUI = true;

	while (System::Update())
	{
		if (prevIndex != index || isForceUpdate)
		{
			switch (index)
			{
			case 0u: displayTexture.fill(inputImage); break;
			case 1u: FillImageFromArrayDouble(depthLayer); break;
			case 2u:
			case 3u:
				displayTexture.fill(inputImage); break;
			}
			prevIndex = index;
			isForceUpdate = false;
		}

		if (prevIndex != 3u)
		{
			displayTexture.draw();
		}
		else
		{
			// resource
			cb->m_lightColor = lightColor.toColorF().rgb();
			cb->m_lightDirection = lightDir.normalized();

			// Lighting
			Graphics2D::SetPSConstantBuffer(1, cb);
			Graphics2D::SetPSTexture(1, resultTexture);
			const ScopedCustomShader2D shader{ pixelShader };
			displayTexture.draw();
		}

		if (isUI)
		{
			// Select Image
			SimpleGUI::RadioButtons(index, texts, Vec2{ 500,10 });

			// Base/Detail
			SimpleGUI::Slider(U"Base", baseWeight, 0.0, 1.0, Vec2{ 500,250 });
			SimpleGUI::Slider(U"Detail", detailWeight, 0.0, 1.0, Vec2{ 500,290 });

			if (SimpleGUI::Button(U"Reconstruct", Vec2{ 500, 470 }))
			{
				ChangeBaseValue();
				ReconstrcutNormalProcess();
				isForceUpdate = true;
			}

			if (prevIndex == 3u)
			{
				SimpleGUI::ColorPicker(lightColor, Vec2{ 500, 400 });
				SimpleGUI::Slider(U"X", lightDir.x, -1.0, 1.0, Vec2{ 10, 450 });
				SimpleGUI::Slider(U"Y", lightDir.y, -1.0, 1.0, Vec2{ 10, 490 });
				SimpleGUI::Slider(U"Z", lightDir.z, -1.0, 1.0, Vec2{ 10, 540 });
			}
		}

		if (KeySpace.down())
		{
			isUI = !isUI;
		}
	}
}

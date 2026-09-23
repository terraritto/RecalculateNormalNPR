#pragma once
#include "Siv3D.hpp"

void LumoSample()
{
	Image inputImage{ U"example/star.png" };
	Image resultImage = inputImage;
	DynamicTexture displayTexture{ inputImage };
	const int ImageWidth = inputImage.width();
	const int ImageHeight = inputImage.height();

	Array<Array<double>> silhoutteNormal;
	silhoutteNormal.resize(ImageHeight);
	for (int i = 0; i < silhoutteNormal.size(); i++) { silhoutteNormal[i].resize(ImageWidth, 0.0); }
	auto gx = silhoutteNormal, gy = silhoutteNormal;

	Array<Array<Vec3>> initialNormal;
	initialNormal.resize(ImageHeight);
	for (int i = 0; i < initialNormal.size(); i++) { initialNormal[i].resize(ImageWidth, Vec3::Zero()); }
	Array<Array<Vec3>> convergenceNormal = initialNormal;

	constexpr int GaussianFilterKernelSize = 1;

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
							const double weight = weights[h+2][w+2];
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
					s[j][i] = ColorF{ Abs(src[j][i]), 0.0, 0.0, 1.0};
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

	const auto ResultPngPath = FilePathView(U"Dump/result.png");
	const auto ResultWebpPath = FilePathView(U"Dump/result.webp");
	auto DumpImage = [&](const Array<Array<Vec3>>& src)
		{
			for (int i = 0; i < ImageWidth; i++)
			{
				for (int j = 0; j < ImageHeight; j++)
				{
					Vec3 r = src[j][i] * 0.5 + Vec3::One() * 0.5;
					resultImage[j][i] = ColorF{ r.x, r.y, r.z, silhoutteNormal[j][i] };
				}
			}
			resultImage.savePNG(ResultPngPath);
			resultImage.saveWebP(ResultWebpPath, Lossless::No, 75);
		};

	auto SilhoutteNormalProcess = [&]()
		{
			auto r = inputImage;
			// AlphaをRに変換 
			for (int i = 0; i < ImageWidth; i++)
			{
				for (int j = 0; j < ImageHeight; j++)
				{
					r[j][i] = ColorF{ static_cast<double>(inputImage[j][i].r), 0.0, 0.0, 1.0 };
				}
			}

			// Gaussian Blurを適用してぼかす
			r.gaussianBlur(GaussianFilterKernelSize);

			// 後々の計算を楽にするためArrayで変換
			for (int i = 0; i < ImageWidth; i++)
			{
				for (int j = 0; j < ImageHeight; j++)
				{
					silhoutteNormal[j][i] = r[j][i].r / 255.0;
				}
			}

			// Sobelで輪郭抽出
			gx = SobelFilter(silhoutteNormal, weightsX);
			gy = SobelFilter(silhoutteNormal, weightsY);

			for (int i = 0; i < ImageWidth; i++)
			{
				for (int j = 0; j < ImageHeight; j++)
				{
					auto x = gx[j][i];
					auto y = gy[j][i];
					auto n = silhoutteNormal[j][i];

					Vec3 g = { -x,y,1.0 };
					initialNormal[j][i] = g.normalized();
				}
			}
;		};

	enum class Method { Jacobi, GaussSeidel, SOR };
	const Method method = Method::SOR;

#pragma region Method
	auto JacobiMethod = [&]()
		{
			auto prev = initialNormal;
			auto current = prev;

			for (int i = 0; i < ImageWidth; i++)
			{
				for (int j = 0; j < ImageHeight; j++)
				{
					if (silhoutteNormal[j][i] > 20.0 / 255.0)
					{
						prev[j][i] = Vec3::Zero();
						continue;
					}

				}
			}

			double delta = 1e+2;
			constexpr double Convegence = 1e-7;

			const Array<Vector2D<int>> direction = { {-1,0},{1,0},{0,-1},{0,1} };

			// ラプラシアンで最適化を行う
			while (delta > Convegence)
			{
				double maxValue = std::numeric_limits<double>::min();
				// ヤコビ法で更新
				for (int i = 0; i < ImageWidth; i++)
				{
					for (int j = 0; j < ImageHeight; j++)
					{
						// 境界条件,外側は更新しない
						if (silhoutteNormal[j][i] < 20.0 / 255.0)
						{
							current[j][i] = prev[j][i];
							continue;
						}

						Vec3 sum = Vec3::Zero();

						for (const auto& dir : direction)
						{
							const int width = Clamp(i + dir.x, 0, ImageWidth - 1);
							const int height = Clamp(j + dir.y, 0, ImageHeight - 1);
							sum += prev[height][width]; // Jacobiはちゃんと現在の計算の影響を入れないように考慮
						}
						current[j][i] = sum / 4.0;

						maxValue = Max(maxValue, (current[j][i] - prev[j][i]).length());
					}
				}

				// 計算した結果で更新
				delta = maxValue;
				prev = current;
			}

			convergenceNormal = prev;
		};

	auto GaussSeidelMethod = [&]()
		{
			auto current = initialNormal;

			for (int i = 0; i < ImageWidth; i++)
			{
				for (int j = 0; j < ImageHeight; j++)
				{
					if (silhoutteNormal[j][i] > 20.0 / 255.0)
					{
						current[j][i] = Vec3::Zero();
						continue;
					}

				}
			}
			auto prev = initialNormal;

			double delta = 1e+2;
			constexpr double Convegence = 1e-7;

			const Array<Vector2D<int>> direction = { {-1,0},{1,0},{0,-1},{0,1} };

			// ラプラシアンで最適化を行う
			while (delta > Convegence)
			{
				double maxValue = std::numeric_limits<double>::min();
				for (int i = 0; i < ImageWidth; i++)
				{
					for (int j = 0; j < ImageHeight; j++)
					{
						// 境界条件,外側は更新しない
						if (silhoutteNormal[j][i] < 20.0 / 255.0)
						{
							continue;
						}

						Vec3 sum = Vec3::Zero();

						for (const auto& dir : direction)
						{
							const int width = Clamp(i + dir.x, 0, ImageWidth - 1);
							const int height = Clamp(j + dir.y, 0, ImageHeight - 1);
							sum += current[height][width]; // Gauss Seidelではその場更新
						}
						current[j][i] = sum / 4.0;

						maxValue = Max(maxValue, (current[j][i] - prev[j][i]).length());
					}
				}

				// 計算した結果で更新
				delta = maxValue;
				prev = current;
			}

			convergenceNormal = prev;
		};

	auto SORMethod = [&]()
		{
			double lambda = 0.5 * (Cos(Math::Pi / ImageWidth) + Cos(Math::Pi / ImageHeight));
			double omega = 2.0 / (1.0 + Sqrt(1.0 + 1.0 - lambda * lambda));

			auto prev = initialNormal;
			auto current = prev;

			for (int i = 0; i < ImageWidth; i++)
			{
				for (int j = 0; j < ImageHeight; j++)
				{
					if (silhoutteNormal[j][i] > 20.0 / 255.0)
					{
						prev[j][i] = Vec3::Zero();
						continue;
					}

				}
			}

			double delta = 1e+2;
			constexpr double Convegence = 1e-7;

			const Array<Vector2D<int>> direction = { {-1,0},{1,0},{0,-1},{0,1} };

			// ラプラシアンで最適化を行う
			while (delta > Convegence)
			{
				double maxValue = std::numeric_limits<double>::min();
				for (int i = 0; i < ImageWidth; i++)
				{
					for (int j = 0; j < ImageHeight; j++)
					{
						// 境界条件,外側は更新しない
						if (silhoutteNormal[j][i] < 20.0 / 255.0)
						{
							current[j][i] = prev[j][i];
							continue;
						}

						Vec3 sum = Vec3::Zero();

						for (const auto& dir : direction)
						{
							const int width = Clamp(i + dir.x, 0, ImageWidth - 1);
							const int height = Clamp(j + dir.y, 0, ImageHeight - 1);
							sum += current[height][width];
						}

						sum = current[j][i] + (sum/4.0 - current[j][i]) * omega;
						current[j][i] = sum;

						maxValue = Max(maxValue, (current[j][i] - prev[j][i]).length());
					}
				}

				// 計算した結果で更新
				delta = maxValue;
				prev = current;
			}

			convergenceNormal = prev;
		};
#pragma endregion

	auto EstimateNormalProcess = [&]()
		{
			switch (method)
			{
			case Method::Jacobi:
				JacobiMethod();
				break;
			case Method::GaussSeidel:
				GaussSeidelMethod();
				break;
			case Method::SOR:
				SORMethod();
				break;
			}
		};

	SilhoutteNormalProcess();

	constexpr bool IsEstimateNormal = true;

	if (IsEstimateNormal)
	{
		EstimateNormalProcess();
		DumpImage(convergenceNormal);
	}
	else
	{
		resultImage = Image{ ResultPngPath };
		for (int i = 0; i < ImageWidth; i++)
		{
			for (int j = 0; j < ImageHeight; j++)
			{
				convergenceNormal[j][i] =
				{
					resultImage[j][i].r / 255.0 * 2.0 - 1.0,
					resultImage[j][i].g / 255.0 * 2.0 - 1.0,
					resultImage[j][i].b / 255.0 * 2.0 - 1.0
				};
			}
		}
	}

	Array<String> texts{ U"Input", U"Silhoutte", U"gx", U"gy", U"InitialNormal", U"ConversienceNormal", U"Lighting" };
	size_t prevIndex = 0;
	size_t index = 0;

	struct ShaderParam
	{
		Vector3D<float> m_lightColor = { 1.0f,1.0f,1.0f };
		Vector3D<float> m_lightBase = { 1.0f,1.0f,1.0f };
		Vector3D<float> m_lightDirection = { 0.0f,0.0f,1.0f };
	};

	const PixelShader pixelShader = HLSL{ U"example/shader/hlsl/lumo_lighting.hlsl", U"PS" };
	ConstantBuffer<ShaderParam> cb;

	HSV lightColor = { 1,1,1 };
	HSV lightBase = { 0.5,1,1 };
	Vec3 lightDir = { 0.0,0.0,1.0 };

	while (System::Update())
	{
		if (prevIndex != index)
		{
			switch (index)
			{
			case 0u: displayTexture.fill(inputImage); break;
			case 1u: FillImageFromArrayDouble(silhoutteNormal); break;
			case 2u: FillImageFromArrayDouble(gx); break;
			case 3u: FillImageFromArrayDouble(gy); break;
			case 4u: FillImageFromArrayVec3(initialNormal); break;
			case 5u: FillImageFromArrayVec3(convergenceNormal); break;
			case 6u: displayTexture.fill(resultImage); break;
			}
			prevIndex = index;
		}

		if (prevIndex != 6u)
		{
			displayTexture.draw();
		}
		else
		{
			// resource
			cb->m_lightColor = lightColor.toColorF().rgb();
			cb->m_lightBase = lightBase.toColorF().rgb();
			cb->m_lightDirection = lightDir.normalized();

			// Lighting
			Graphics2D::SetPSConstantBuffer(1, cb);
			const ScopedCustomShader2D shader{ pixelShader };
			displayTexture.draw();
		}

		if (prevIndex == 6u)
		{
			SimpleGUI::ColorPicker(lightColor, Vec2{ 500, 400 });
			SimpleGUI::ColorPicker(lightBase, Vec2{ 600, 400 });
			SimpleGUI::Slider(U"X", lightDir.x, -1.0, 1.0, Vec2{10, 450});
			SimpleGUI::Slider(U"Y", lightDir.y, -1.0, 1.0, Vec2{10, 490});
			SimpleGUI::Slider(U"Z", lightDir.z, -1.0, 1.0, Vec2{10, 540});
		}

		SimpleGUI::RadioButtons(index, texts, Vec2{ 500,10 });
	}
}

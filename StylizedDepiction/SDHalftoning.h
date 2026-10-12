#pragma once
#include <Siv3D.hpp>

void SDHalftoning()
{
	Image inputImage{ U"example/ship_01.jpg" };
	Image L = inputImage;
	DynamicTexture displayTexture{ inputImage };
	const int ImageWidth = inputImage.width();
	const int ImageHeight = inputImage.height();

	Array<Array<double>> depthLayer;
	depthLayer.resize(ImageHeight);
	for (int i = 0; i < depthLayer.size(); i++) { depthLayer[i].resize(ImageWidth, 0.0); }
	auto inputLayer = depthLayer;
	
	auto result = depthLayer;
	Vec3 pointLight = { 100, 100, 2.0 };

	auto f = [](const double& x)
		{
			return (3.0 + (-6.0 + 4.0 * x) * x) * x;
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

	auto HalfToneing = [&](const Array<Array<double>>& src)
		{
			auto s = inputImage;
			for (int i = 0; i < ImageWidth; i++)
			{
				for (int j = 0; j < ImageHeight; j++)
				{
					s[j][i] = ColorF{ src[j][i], src[j][i], src[j][i], 1.0 };
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

	auto GetDDAPoints = [](Vec2 p1, Vec2 p2)
		{
			Array<Vec2> points;
			Vec2 d = p2 - p1;
			double step = Abs(d.x) >= Abs(d.y) ? Abs(d.x) : Abs(d.y);

			// 0の場合は完全に1点のみなので、開始地点はいれない
			if (step <= 0.0)
			{
				return points;
			}

			d /= step;

			int count = 0;
			while (count <= step)
			{
				points.push_back(p1);
				p1 += d;
				count++;
			}

			points.pop_front(); // 最初の点だけ除いておく
			return points;
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

	auto HalfToneProcess = [&]()
		{
			// AlphaをRに変換 
			for (int i = 0; i < ImageWidth; i++)
			{
				for (int j = 0; j < ImageHeight; j++)
				{
					Vec2 from = { i, j }; Vec2 to = { pointLight.xy() };
					const double targetDistance = from.distanceFrom(to);
					auto points = GetDDAPoints(from, to);

					bool isIntersect = false;
					for (const auto& point : points)
					{
						double depth = depthLayer[point.y][point.x];

						// 現状の高さに置ける値をlerpで計算してみる
						double pointDistance = from.distanceFrom(point.xy());
						double threshold = pointDistance / targetDistance;
						double currentDepth = Math::Lerp(depthLayer[j][i], pointLight.z, threshold);

						// 交差してる？
						if (currentDepth <= depth)
						{
							isIntersect = true;
							break;
						}
					}

					result[j][i] = isIntersect ? 0.0 : 1.0;
				}
			}
		};

	RecoverNormalProcess();
	ChangeBaseValue();
	HalfToneProcess();

	Array<String> texts{ U"Input", U"Depth", U"HalfTone" };
	size_t prevIndex = 0;
	size_t index = 0;

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
			case 2u: HalfToneing(result); break;
			}
			prevIndex = index;
			isForceUpdate = false;
		}

		displayTexture.draw();

		if (isUI)
		{
			// Select Image
			SimpleGUI::RadioButtons(index, texts, Vec2{ 500,10 });

			// Base/Detail
			bool isChangeWeight = false;
			isChangeWeight |= SimpleGUI::Slider(U"Base", baseWeight, 0.0, 1.0, Vec2{ 500,250 });
			isChangeWeight |= SimpleGUI::Slider(U"Detail", detailWeight, 0.0, 1.0, Vec2{ 500,290 });
			if (isChangeWeight) { ChangeBaseValue(); isForceUpdate = true; }

			// HalfTone
			SimpleGUI::Slider(U"PointX:", pointLight.x, 0.0, ImageWidth, Vec2{ 500,350 });
			SimpleGUI::Slider(U"PointY:", pointLight.y, 0.0, ImageHeight, Vec2{ 500,390 });
			SimpleGUI::Slider(U"Depth:", pointLight.z, 0.0, 2.0, Vec2{ 500,430 });
			if (SimpleGUI::Button(U"HalfToneing", Vec2{ 500, 470 }))
			{
				HalfToneProcess();
				isForceUpdate = true;
			}
		}

		if (KeySpace.down())
		{
			isUI = !isUI;
		}
	}
}

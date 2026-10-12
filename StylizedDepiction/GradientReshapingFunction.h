#pragma once
#include <Siv3D.hpp>

void GradientReshapingFunction()
{
	Scene::SetBackground(Palette::White);

	constexpr int offset = 15;

	auto sceneSize = Scene::Size();
	Vec2 sceneMin = Vec2{ offset,offset };
	Vec2 sceneMax = sceneSize - Vec2{ offset,offset };

	auto f = [](const double& x)
		{
			return (3.0 + (-6.0 + 4.0 * x) * x) * x;
		};

	Array<Array<double>> results;

	constexpr int steps = 1000;
	for (int j = 0; j < 4; j++)
	{
		Array<double> result;
		for (int i = 0; i < steps; i++)
		{
			if (j != 0)
			{
				result.push_back(f(results[j-1][i]));
			}
			else
			{
				double x = i / static_cast<double>(steps - 1);
				result.push_back(f(x));
			}
		}
		results.push_back(result);
	}

	auto DrawValues = [&](const Array<double>& values, const ColorF color)
		{
			for (int i = 0; i < values.size() - 1; i++)
			{
				Vec2 start{ i / static_cast<double>(steps - 1), 1.0 - values[i] };
				start.x = Math::Lerp(sceneMin.x, sceneMax.x, start.x);
				start.y = Math::Lerp(sceneMin.y, sceneMax.y, start.y);
				Vec2 end{ (i+1) / static_cast<double>(steps - 1), 1.0 - values[i + 1]};
				end.x = Math::Lerp(sceneMin.x, sceneMax.x, end.x);
				end.y = Math::Lerp(sceneMin.y, sceneMax.y, end.y);

				Line{ start, end }.draw(color);
			}
		};

	while (System::Update())
	{
		for (int i = 0; i < results.size(); i++)
		{
			ColorF color = Colormap01F(i / static_cast<double>(results.size() - 1), ColormapType::Turbo);
			DrawValues(results[i], color);
		}
	}
}

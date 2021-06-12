#pragma once

#include <glm/glm.hpp>
#include <stdint.h>
#include <math.h>

#include <iostream>

template<typename T>
class Texture {
public:
	Texture(void) : Data(nullptr), Width(0), Height(0) {}

	using Pixel = T;
	const static size_t PixelSize = sizeof(Pixel);

	using PixelCoordinate = const size_t;

	void Create(PixelCoordinate W, PixelCoordinate H) {
		Width = W;
		Height = H;
		Data = new Pixel[W * H];
	}

	void Resize(PixelCoordinate W, PixelCoordinate H) {
		Texture Old = *this;
		Create(W, H);
		Copy(&Old, this);
		Old.Free();
	}

	// Copies T0 to T1 starting at (0, 0)
	void Copy(Texture* T0, Texture* T1) {
		for (size_t Y = 0; Y < T0->Width; Y++) {
			for (size_t X = 0; X < T0->Width; X++) {
				T1->Write(X, Y, T0->Read(X, Y));
			}
		}
	}

	void Free(void) {
		delete[] Data;
	}

	// Returns with refrence 
	Pixel& GetPixel(PixelCoordinate X, PixelCoordinate Y) {
		return Data[GetTextureIndex(X, Y)];
	}

	void Write(PixelCoordinate X, PixelCoordinate Y, const Pixel& Value) {
		Data[GetTextureIndex(X, Y)] = Value;
	}

	void WriteFlipped(PixelCoordinate X, PixelCoordinate Y, const Pixel& Value) {
		Write(X, Height - Y - 1, Value);
	}

	Pixel Read(PixelCoordinate X, PixelCoordinate Y) const  {
		return Data[GetTextureIndex(X, Y)];
	}

	// Currently only nearest neighbor sampling
	Pixel Sample(const glm::vec2& UV) {
		glm::ivec2 Coords = glm::ivec2(glm::round(UV * glm::vec2(Width, Height)));
		return Read((PixelCoordinate)Coords.x, (PixelCoordinate)Coords.y);
	}

	size_t GetTextureIndex(PixelCoordinate X, PixelCoordinate Y) const {
		if (!ValidateCoordinates(X, Y)) {
			return 0;
		}
		return Y * Width + X;
	}

	Pixel* GetRawData(void) {
		return Data;
	}

	Pixel* GetRawDataConstant(void) const {
		return Data;
	}

	void Clear(const Pixel& Value) {
		for (size_t Y = 0; Y < Height; Y++) {
			for (size_t X = 0; X < Width; X++) {
				Write(X, Y, Value);
			}
		}
	}

	size_t GetWidth(void) const {
		return Width;
	}

	size_t GetHeight(void) const {
		return Height;
	}

	void SetSize(PixelCoordinate W, PixelCoordinate H) {
		if (Width != W || Height != H) {
			if (Data) {
				Resize(W, H);
			}
			else {
				Create(W, H);
			}
		}
	}

	// Texture filter used
	enum class FilteringMode {
		AUTO,    // Default to texture's current filtering mode
		NEAREST, // Nearest neighbor sampling
		LINEAR,  // Linear filtering
	};

	//template<FilteringMode Filter = FilteringMode::AUTO>
	//Pixel Sample(float X, float Y);

	/*
	template <FilteringMode::NEAREST>
	Pixel Sample(float X, float Y) {
		size_t PX = (size_t)roundf(X * Width);
		size_t PY = (size_t)roundf(Y * Height);
		
		return Read(PX, PY);
	}
	*/

private:
	Pixel* Data;
	size_t Width;
	size_t Height;

	FilteringMode Filter;

	bool ValidateCoordinates(PixelCoordinate X, PixelCoordinate Y) const {
		bool ValidX = X < Width;
		bool ValidY = Y < Height;
		if (ValidX && ValidY) {
			return true;
		} else {
			std::cout << "WARNING: INVALID COORDINATES TO WRITE TO:\t " << X << '\t ' << Y << "\t WIDTH AND HEIGHT:\t " << Width << "\t " << Height << '\n';
			return false;
		}
	}
};
#include "Image.h"
#include <fstream>
#include <sstream>
#include <vector>
#include <string>
#include <algorithm>

namespace ImageIO {

	// You can try more 9s but be careful of floating point errors, MSVC at most allows 255.9999999999999 before rounding up to 256
	constexpr float F32to8UI = 255.9999f;

	struct ByteSerializer {
		constexpr static size_t kColorRange = 255;
		std::vector<std::string> SerializationLUT;

		ByteSerializer(void) {
			SerializationLUT.reserve(kColorRange);
			for (uint32_t Num = 0; Num < kColorRange + 1; Num++) {
				SerializationLUT.push_back(std::to_string(Num));
			}
		}

		std::string& Serialize(uint8_t Value) {
			return SerializationLUT[(size_t)Value];
		}
	};

	static ByteSerializer Serializer;

	uint8_t TransformColor(float X) {
		X = glm::clamp(X, 0.0f, 1.0f);
		if (X <= 0.0031308f) {
			X *= 12.92f;
		}
		else {
			X = 1.055f * pow(X, 1.0f / 2.4f) - 0.055f;
		}
		X *= F32to8UI;
		return (uint8_t)X;
	}

	void CreateImage(const char* Path, Texture<ColorRGB32F>& ImageData) {
		// Reserve memory for text buffer
		std::string ReservedMemory;
		ReservedMemory.reserve((4 + 4 + 4) * ImageData.GetHeight() * ImageData.GetWidth() + 256);
		std::ostringstream TextBuffer(ReservedMemory);
		// Create header of PPM file
		TextBuffer << "P3\n" << ImageData.GetWidth() << ' ' << ImageData.GetHeight() << "\n255\n";
		// Write image data
		for (size_t Y = 0; Y < ImageData.GetHeight(); Y++) {
			for (size_t X = 0; X < ImageData.GetWidth(); X++) {
				ColorRGB32F& CurrentColor = ImageData.GetPixel(X, Y);
				// Convert to [0, 255] domain
				ColorRGB8UI ByteColor;
				ByteColor.r = TransformColor(CurrentColor.r);
				ByteColor.g = TransformColor(CurrentColor.g);
				ByteColor.b = TransformColor(CurrentColor.b);
				// Write color (I could use a binary format here but human readability is useful sometimes)
				TextBuffer
					<< Serializer.Serialize(ByteColor.r)
					<< ' '
					<< Serializer.Serialize(ByteColor.g)
					<< ' '
					<< Serializer.Serialize(ByteColor.b)
					<< (X == ImageData.GetWidth() ? '\n' : ' ');
			}
		}
		// Write to file
		std::ofstream ImageOutput(Path);
		ImageOutput << TextBuffer.str();
		ImageOutput.flush();
		ImageOutput.close();
	}

}
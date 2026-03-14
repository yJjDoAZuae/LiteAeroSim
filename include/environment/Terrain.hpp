#pragma once

namespace liteaerosim::environment {

class V_Terrain {
public:
    [[nodiscard]] virtual float elevation_m(double latitude_rad,
                                            double longitude_rad) const = 0;

    [[nodiscard]] float heightAboveGround_m(float  altitude_m,
                                            double latitude_rad,
                                            double longitude_rad) const;

    virtual ~V_Terrain() = default;
};

class FlatTerrain : public V_Terrain {
public:
    explicit FlatTerrain(float elevation_m = 0.f);

    [[nodiscard]] float elevation_m(double latitude_rad,
                                    double longitude_rad) const override;

private:
    float elevation_m_;
};

} // namespace liteaerosim::environment

class PerlinNoise {
    constructor() {
        this.p_perm = [];
        for (let i = 0; i < 256; i++) {
            this.p_perm[i] = i;
        }
        // Shuffle p_perm (Fisher-Yates)
        for (let i = this.p_perm.length - 1; i > 0; i--) {
            const j = Math.floor(Math.random() * (i + 1));
            [this.p_perm[i], this.p_perm[j]] = [this.p_perm[j], this.p_perm[i]];
        }
        this.permutation = [...this.p_perm, ...this.p_perm]; // Double the permutation array
    }

    fade(t) {
        return t * t * t * (t * (t * 6 - 15) + 10);
    }

    lerp(a, b, t) {
        return a + t * (b - a);
    }

    grad1D(hash, x) {
        return (hash & 1) === 0 ? x : -x;
    }

    noise1D(x) {
        const xi = Math.floor(x) & 255;
        const xf = x - Math.floor(x);
        const u = this.fade(xf);

        const g1 = this.grad1D(this.permutation[xi], xf);
        const g2 = this.grad1D(this.permutation[xi + 1], xf - 1);

        return this.lerp(g1, g2, u);
    }
}

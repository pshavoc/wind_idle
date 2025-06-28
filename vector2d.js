class Vector2D {
    constructor(x, y) {
        this.x = x;
        this.y = y;
    }

    normalize() {
        const len = this.length();
        if (len > 0) {
            this.x /= len;
            this.y /= len;
        }
        return this;
    }

    clone() {
        return new Vector2D(this.x, this.y);
    }

    length() {
        return Math.sqrt(this.x * this.x + this.y * this.y);
    }

    dot(otherVector) {
        return this.x * otherVector.x + this.y * otherVector.y;
    }

    distanceTo(otherVector) {
        const dx = this.x - otherVector.x;
        const dy = this.y - otherVector.y;
        return Math.sqrt(dx * dx + dy * dy);
    }

    add(otherVector) {
        this.x += otherVector.x;
        this.y += otherVector.y;
        return this;
    }

    subtract(otherVector) {
        this.x -= otherVector.x;
        this.y -= otherVector.y;
        return this;
    }

    addScalar(scalar) {
        this.x += scalar;
        this.y += scalar;
        return this;
    }

    multiplyScalar(scalar) {
        this.x *= scalar;
        this.y *= scalar;
        return this;
    }

    divideScalar(scalar) {
        if (scalar !== 0) {
            this.x /= scalar;
            this.y /= scalar;
        } else {
            this.x = 0;
            this.y = 0;
        }
        return this;
    }

    angle() {
        return Math.atan2(this.y, this.x);
    }

    angleTo(otherVector) {
        const dotProduct = this.dot(otherVector);
        const lenProduct = this.length() * otherVector.length();
        if (lenProduct === 0) {
            return 0; // Or handle as an error, e.g., throw an exception
        }
        return Math.acos(dotProduct / lenProduct);
    }
}

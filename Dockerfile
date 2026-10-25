# syntax=docker/dockerfile:1
# Multi-stage build using cargo-chef for dependency-layer caching (works with the crates/* glob).
FROM rust:1-bookworm AS chef
RUN cargo install cargo-chef --locked --version 0.1.71
WORKDIR /app

FROM chef AS planner
COPY Cargo.toml Cargo.lock ./
COPY crates ./crates
RUN cargo chef prepare --recipe-path recipe.json

FROM chef AS builder
COPY --from=planner /app/recipe.json recipe.json
RUN cargo chef cook --release -p shot-service --recipe-path recipe.json
COPY Cargo.toml Cargo.lock ./
COPY crates ./crates
RUN cargo build --release -p shot-service

FROM gcr.io/distroless/cc-debian12:nonroot AS runtime
COPY --from=builder /app/target/release/shot-service /usr/local/bin/shot-service
ENV LISTEN_ADDR=0.0.0.0:8080 RUST_LOG=info
EXPOSE 8080
USER nonroot:nonroot
ENTRYPOINT ["/usr/local/bin/shot-service"]

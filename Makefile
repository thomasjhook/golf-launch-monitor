CARGO := cargo
IMAGE ?= launch-monitor/shot-service:dev
K8S_DIR := deploy/k8s

DATABASE_URL ?= postgres://launch:launch-dev@127.0.0.1:5432/launchmonitor

.PHONY: build test test-db db-up db-down db-reset clippy fmt docker-build docker-run k8s-render k8s-secret k8s-apply k8s-delete

build:
	$(CARGO) build --workspace
test:
	$(CARGO) test --workspace
test-db: db-up
	DATABASE_URL=$(DATABASE_URL) $(CARGO) test -p shot-service --test postgres -- --ignored --test-threads=1

# Local Postgres for native runs: DATABASE_URL=$(DATABASE_URL) cargo run -p shot-service
db-up:
	docker compose up -d --wait postgres
db-down:
	docker compose down
db-reset:
	docker compose down -v

clippy:
	$(CARGO) clippy --workspace --all-targets -- -D warnings
fmt:
	$(CARGO) fmt --all

docker-build:
	docker build -t $(IMAGE) .
docker-run:
	docker run --rm -p 8080:8080 -e RUST_LOG=info $(IMAGE)

k8s-render:
	kubectl kustomize $(K8S_DIR)

# LOCAL CLUSTERS ONLY: refuses unless context is docker-desktop, kind-*, or minikube.
define check_ctx
@ctx=$$(kubectl config current-context 2>/dev/null); \
case "$$ctx" in docker-desktop|kind-*|minikube) ;; \
*) echo "Refusing: current context '$$ctx' is not a local cluster (docker-desktop/kind-*/minikube)"; exit 1;; esac
endef

# Creates the namespace and a random Postgres password, once. The password is
# never written to the repo; delete the Secret to rotate (and reset the DB).
k8s-secret:
	$(check_ctx)
	kubectl create namespace launch-monitor --dry-run=client -o yaml | kubectl apply -f -
	kubectl -n launch-monitor get secret postgres-credentials >/dev/null 2>&1 || \
	  kubectl -n launch-monitor create secret generic postgres-credentials \
	    --from-literal=password=$$(openssl rand -hex 24)

k8s-apply: k8s-secret
	$(check_ctx)
	kubectl apply -k $(K8S_DIR)
k8s-delete:
	$(check_ctx)
	kubectl delete -k $(K8S_DIR)
